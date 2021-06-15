package xWRDataPreProc

import chisel3._
import chisel3.util._
import chisel3.experimental._

import dsptools._
import dsptools.numbers._

import dspblocks._
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.amba.axi4stream._
import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper._
import freechips.rocketchip.tilelink._


trait AXI4xWRDataPreProcStandaloneBlock extends AXI4xWRdataPreProcBlock {
  def standaloneParams = AXI4BundleParameters(addrBits = 32, dataBits = 32, idBits = 1)
  val ioMem = mem.map { m => {
    val ioMemNode = BundleBridgeSource(() => AXI4Bundle(standaloneParams))

    m :=
      BundleBridgeToAXI4(AXI4MasterPortParameters(Seq(AXI4MasterParameters("bundleBridgeToAXI4")))) :=
      ioMemNode

    val ioMem = InModuleBody { ioMemNode.makeIO() }
    ioMem
  }}
  
  val ioInNode = BundleBridgeSource(() => new AXI4StreamBundle(AXI4StreamBundleParameters(n = 2)))
  val ioOutNode = BundleBridgeSink[AXI4StreamBundle]()

  ioOutNode :=
    AXI4StreamToBundleBridge(AXI4StreamSlaveParameters()) :=
    streamNode :=
    BundleBridgeToAXI4Stream(AXI4StreamMasterParameters(n = 4)) :=
    ioInNode

  val in = InModuleBody { ioInNode.makeIO() }
  val out = InModuleBody { ioOutNode.makeIO() }
}


abstract class XWRdataPreProcBlock [ D, U, E, O, B <: Data] (beatBytes: Int, maxFFTSize: Int, queueSize: Int) extends LazyModule()(Parameters.empty) with DspBlock[D, U, E, O, B] with HasCSR {
  // This core should also generate lastOut signal depeneding on number of frames, number of chirps etc.
  
  val masterParams = AXI4StreamMasterParameters(
    name = "AXI4 Stream xWrDataPreProc",
    n = 4, // 4*8 -> 32 bits
    numMasters = 1
  )
  val slaveParams = AXI4StreamSlaveParameters()
  
  val slaveNode  = AXI4StreamSlaveNode(slaveParams)
  val masterNode = AXI4StreamMasterNode(masterParams)
  // one another master node should be added for config
  // ready signal is  ready signal from fft core
  // but that ready signal is actually check that fft core is in state idle
  // only when fft is in the state idle configuration can be applied!
  
  val streamNode = NodeHandle(slaveNode, masterNode)
  
  lazy val module = new LazyModuleImp(this) {

    val (out, edgeOut) = masterNode.out.head
    val (in, edgeIn) = slaveNode.in.head
    
    val CP_data_length = 0                 // in the future equal to 2
    val log2fftSize = log2Up(maxFFTSize + 1)
    
    // Define Control registers
    val IQSwap = RegInit(false.B)          // default is 0, I first
    val adcFormat = RegInit(0.U(2.W))      // default is 0, Real
    val testPattern = RegInit(true.B)      // default is 1, change in future
    val rawData = RegInit(true.B)          // rawData is on, dsp part is off
    // val msbFirst = RegInit(true)        // default is 1, msb first is
    // val crcEnable = RegInit(false.B)    // default is off, crcEnable is off
    val delayedInData = RegInit(0.U(16.W))
    val swap = RegInit(false.B)
    val delayedOutData = RegNext(out.bits.data)
    
    val fftSize      = RegNext(maxFFTSize.U(log2fftSize.W))
    val adcSamplesP0 = RegInit(maxFFTSize.U(log2fftSize.W))
    val adcSamplesP1 = RegInit(maxFFTSize.U(log2fftSize.W))
    val adcSamplesP2 = RegInit(maxFFTSize.U(log2fftSize.W))
    val adcSamplesP3 = RegInit(maxFFTSize.U(log2fftSize.W))
    
    // fftSize needs to be defined inside this preproc block,
    // not configured via memory master model but for initial testing
    // it is defined on that way and it is assumed that only profile used in radar configuration is profile 0
    // when streaming interface for fft configuration is inserted then fftSize should be configured only when previous CP is different than current CP value
    
    // val numZeros     = RegInit((maxFFTSize/2).U(log2Up(maxFFTSize/2 + 1)))
    // this in the future should replace switch where condition for switch is actually content of the field2 of CP data
    val adcSamples = adcSamplesP0
    assert(fftSize >= adcSamplesP0, "FFT size needs to be larger than configured number of samples inside chirp")
    val numZeros = fftSize - adcSamples
    val cntInData  = RegInit(0.U(log2Up(maxFFTSize + 1 + CP_data_length).W)) // assume there is no CP inserted, so maxFFTSize should be limit here
    val cntOutData = RegInit(0.U(log2Up(maxFFTSize + 1).W)) // assume there is no CP inserted, so maxFFTSize should be limit here
    //val zeroPaddFlag = RegInit(false.B)
    val zeroPaddFlag = Wire(Bool())
    
    val fields = Seq(
      // settable registers
      RegField(1, IQSwap,
        RegFieldDesc(name = "IQ swap", desc = "I or Q data is first sent")),
      RegField(2, adcFormat,
        RegFieldDesc(name = "adcFormat", desc = "Define adcFormat 0 -> real, 1 -> complex 1x, 2 -> complex")),
      RegField(1, testPattern,
        RegFieldDesc(name = "testPattern", desc = "Enable test pattern mode")),
      RegField(1, rawData,
        RegFieldDesc(name = "rawData", desc = "Enable rawData")),
      RegField(log2fftSize, fftSize,
        RegFieldDesc(name = "fftSize", desc = "Configured size of the FFT")),
      RegField(log2fftSize, adcSamplesP0,
        RegFieldDesc(name = "adcSamplesP0", desc = "Number of samples per chirp for profile 0")),
      RegField(log2fftSize, adcSamplesP1,
        RegFieldDesc(name = "adcSamplesP1", desc = "Number of samples per chirp for profile 1")),
      RegField(log2fftSize, adcSamplesP2,
        RegFieldDesc(name = "adcSamplesP2", desc = "Number of samples per chirp for profile 2")),
      RegField(log2fftSize, adcSamplesP3,
        RegFieldDesc(name = "adcSamplesP3", desc = "Number of samples per chirp for profile 3"))
      // read-only status registers
      //RegField.r(1, msbFirst,
        //RegFieldDesc(name = "msbFirst", desc = ""))
    )
    
    // Define abstract register map so it can be AXI4, Tilelink, APB, AHB
    regmap(fields.zipWithIndex.map({ case (f, i) => i * beatBytes -> Seq(f)}): _*)
    in.ready := out.ready
    val dataQueue = Module(new Queue(UInt((beatBytes*8).W), entries = queueSize, flow = true))
    
    when (out.valid && out.ready) {
      cntOutData := cntOutData + 1.U
    }
    //when (cntOutData === fftSize) {
    when (cntOutData === (fftSize - 1.U)) {
      cntOutData := 0.U
    }
    
    when (cntOutData < adcSamples) {
      zeroPaddFlag := false.B
    }
    .otherwise {
      zeroPaddFlag := true.B
      /*when (cntOutData === (fftSize - 1.U)) {
        zeroPaddFlag := false.B
      }
      .otherwise {
        zeroPaddFlag := true.B
      }*/
    }
    
    /*when (cntOutData === (adcSamples - 1.U)) {
      zeroPaddFlag := true.B
    }
    when (cntOutData === fftSize) { // or fftSize - 1, this can be checked when additional block is added
      zeroPaddFlag := false.B
    }*/
    val inValidReg = RegNext(in.valid)
    val inDataReg = RegNext(in.bits.data)
    
    
    when (testPattern || rawData) {
      // there is no need to do zero padding for those cases
      dataQueue.io.enq.bits   := in.bits.data // lsb bits are filled, by default msb bits are zero 
      dataQueue.io.enq.valid  := in.valid
      dataQueue.io.deq.ready  := out.ready
    }
    .elsewhen (adcFormat === 0.U) {
      dataQueue.io.enq.bits  := inDataReg//in.bits.data // lsb bits are filled, by default msb bits are zero 
      dataQueue.io.enq.valid := inValidReg//in.valid
      dataQueue.io.deq.ready := ~zeroPaddFlag
     // out.bits.data    := Cat(in.bits.data.asUInt, 0.U(16.W)) // lsb bits are filled, by default msb bits are zero
     // out.valid        := in.valid
    }
    .otherwise {
    //.elsewhen (adcFormat === 1.U || adcFormat === 2.U) { // Complex 1-x
      when (inValidReg & ~swap) {
      //when (in.valid & ~swap) {
        delayedInData := inDataReg //in.bits.data
        swap := true.B
        dataQueue.io.enq.bits := 0.U
        dataQueue.io.enq.valid  := false.B
        dataQueue.io.deq.ready := ~zeroPaddFlag//false.B//out.ready // perhaps this is not important at all
      }
      //.elsewhen (in.valid & swap) {
      .elsewhen (inValidReg & swap) {
        swap := false.B
        dataQueue.io.enq.valid := true.B
        when (IQSwap) {
          //dataQueue.io.enq.bits := Cat(delayedInData.asUInt, in.bits.data.asUInt)
          dataQueue.io.enq.bits := Cat(delayedInData.asUInt, inDataReg)
        }
        .otherwise {
          //dataQueue.io.enq.bits := Cat(in.bits.data.asUInt, delayedInData.asUInt)
          dataQueue.io.enq.bits := Cat(inDataReg, delayedInData.asUInt)
        }
        dataQueue.io.deq.ready := ~zeroPaddFlag // this should be generated from logic responsible for zero padding
      }
      .otherwise {
        dataQueue.io.enq.bits  := delayedOutData
        dataQueue.io.enq.valid := false.B
        dataQueue.io.deq.ready := false.B//out.ready // perhaps this is not important at all
      }
    }
    // perhaps dataQueue.io,enq.ready need to be actually just ~zeroPaddFlag
    
    when (zeroPaddFlag) {
      out.valid := true.B
      out.bits.data := 0.U
      in.ready := out.ready
      // but in theory this block should be always ready to accept data
    }
    .otherwise {
      out.valid := dataQueue.io.deq.valid
      out.bits.data := dataQueue.io.deq.bits
      in.ready := dataQueue.io.enq.ready
    }
  }
}

class AXI4xWRdataPreProcBlock(address: AddressSet, _beatBytes: Int = 4, maxFFTSize: Int = 1024, queueSize: Int = 512)(implicit p: Parameters) extends XWRdataPreProcBlock[AXI4MasterPortParameters, AXI4SlavePortParameters, AXI4EdgeParameters, AXI4EdgeParameters, AXI4Bundle](_beatBytes, maxFFTSize, queueSize) with AXI4DspBlock with AXI4HasCSR {
  override val mem = Some(AXI4RegisterNode(address = address, beatBytes = _beatBytes))
}

class TLxWRdataPreProcBlock(address: AddressSet, _beatBytes: Int = 4, maxFFTSize: Int = 1024, queueSize: Int = 512)(implicit p: Parameters) extends XWRdataPreProcBlock[TLClientPortParameters, TLManagerPortParameters, TLEdgeOut, TLEdgeIn, TLBundle](_beatBytes, maxFFTSize, queueSize) with TLDspBlock with TLHasCSR {
  val devname = "TLxWRdataPreProcBlock"
  val devcompat = Seq("xWRDataPreProc", "radardsp")
  val device = new SimpleDevice(devname, devcompat) {
    override def describe(resources: ResourceBindings): Description = {
      val Description(name, mapping) = super.describe(resources)
      Description(name, mapping)
    }
  }
  // make diplomatic TL node for regmap
  override val mem = Some(TLRegisterNode(address = Seq(address), device = device, beatBytes = _beatBytes))
}

object AXI4xWRPreProcBlockApp extends App
{
  val baseAddress = 0x500 // just to check if verilog code is succesfully generated or not
  val maxFFTSize = 1024
  val queueSize = 512
  implicit val p: Parameters = Parameters.empty

  val xWRDataPreProcModule = LazyModule(new AXI4xWRdataPreProcBlock(AddressSet(baseAddress, 0xFF), _beatBytes = 4, maxFFTSize) with AXI4xWRDataPreProcStandaloneBlock)
  chisel3.Driver.execute(args, ()=> xWRDataPreProcModule.module)
}
