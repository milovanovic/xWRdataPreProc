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
import dsputils._


trait AXI4xWRDataPreProcMultipleStreamsStandaloneBlock extends AXI4xWRdataPreProcMultipleStreams {
  def standaloneParams = AXI4BundleParameters(addrBits = 32, dataBits = 32, idBits = 1)
  val ioMem = mem.map { m => {
    val ioMemNode = BundleBridgeSource(() => AXI4Bundle(standaloneParams))

    m :=
      BundleBridgeToAXI4(AXI4MasterPortParameters(Seq(AXI4MasterParameters("bundleBridgeToAXI4")))) :=
      ioMemNode

    val ioMem = InModuleBody { ioMemNode.makeIO() }
    ioMem
  }}


 // self: AXI4StreamCombiner =>
  def nIn:  Int = 4
  def nOut: Int = 4
  def beatBytes: Int = 2

  val inIO: Seq[ModuleValue[AXI4StreamBundle]] = for (i <- 0 until nIn) yield {
    implicit val valName = ValName(s"inIO_$i")
    val in = BundleBridgeSource[AXI4StreamBundle](() => AXI4StreamBundle(AXI4StreamBundleParameters(n = beatBytes)))
     streamNode := BundleBridgeToAXI4Stream(AXI4StreamMasterPortParameters(AXI4StreamMasterParameters(n = beatBytes))) := in
    InModuleBody { in.makeIO() }
  }
  val outIO: Seq[ModuleValue[AXI4StreamBundle]] = for (o <- 0 until nOut) yield {
    implicit val valName = ValName(s"outIO_$o")
    val out = BundleBridgeSink[AXI4StreamBundle]()
    out := AXI4StreamToBundleBridge(AXI4StreamSlavePortParameters(AXI4StreamSlaveParameters())) := streamNode

    InModuleBody { out.makeIO() }
  }
}


abstract class XWRdataPreProcMultipleStreamsBlock [D, U, E, O, B <: Data] (params: AXI4XwrDataPreProcParams, beatBytes: Int) extends LazyModule()(Parameters.empty) with DspBlock[D, U, E, O, B] with HasCSR {
  // This core should also generate lastOut signal depeneding on number of frames, number of chirps etc.

  val streamNode = AXI4StreamNexusNode(
    masterFn = (ms: Seq[AXI4StreamMasterPortParameters]) => AXI4StreamMasterPortParameters(
      Seq(AXI4StreamMasterParameters(
        n = 4 // output is 32 bit
    ))),
    slaveFn = ss => {AXI4StreamSlavePortParameters(ss.map(_.slaves).reduce(_ ++ _))}
  )

  // one another master node should be added for config
  // ready signal is  ready signal from fft core
  // but that ready signal is actually check that fft core is in state idle
  // only when fft is in the state idle configuration can be applied!

  lazy val module = new LazyModuleImp(this) {

    val (ins, _) = streamNode.in.unzip
    val (outs, _) = streamNode.out.unzip

    // val (out, edgeOut) = masterNode.out.head
    // val (in, edgeIn) = slaveNode.in.head

    val CP_data_length = 0                 // in the future equal to 2
    val log2fftSize = log2Up(params.maxFFTSize + 1)
    val log2MaxChirpsPerFrame = log2Up(params.maxChirpsPerFrame + 1)

    // Define Control registers
    val IQSwap = RegInit(true.B)           // default is 1, I first
    val adcFormat = RegInit(1.U(2.W))      // default is 1, complex 1x
    val testPattern = RegInit(false.B)     // default is 0, default is assumed that testPattern mode turned off
    val rawData = RegInit(false.B)         // rawData is of, dsp part is on
    // val msbFirst = RegInit(true)        // default is 1, msb first is
    // val crcEnable = RegInit(false.B)    // default is off, crcEnable is off


    val fftSize        = RegInit(params.maxFFTSize.U(log2fftSize.W))
    val chirpsPerFrame = RegInit(64.U(log2MaxChirpsPerFrame.W))      // valid range 1 to 255 - defined by TI ICD document
    val adcSamplesP0   = RegInit(params.maxFFTSize.U(log2fftSize.W))
    val adcSamplesP1   = RegInit(params.maxFFTSize.U(log2fftSize.W))
    val adcSamplesP2   = RegInit(params.maxFFTSize.U(log2fftSize.W))
    val adcSamplesP3   = RegInit(params.maxFFTSize.U(log2fftSize.W))

    // fftSize needs to be defined inside this preproc block,
    // not configured via memory master model but for initial testing
    // it is defined on that way and it is assumed that the only profile used in radar configuration is profile 0
    // when streaming interface for fft configuration is inserted then fftSize should be configured only when previous CP is different than current CP value
    val genLast = RegInit(true.B)          // by default last signal should be generated
    // val numZeros     = RegInit((maxFFTSize/2).U(log2Up(maxFFTSize/2 + 1)))
    // this in the future should replace switch where condition for switch is actually content of the field2 of CP data
    val adcSamples = adcSamplesP0
    // should not be asserted in config phase
    //assert(fftSize >= adcSamplesP0, "FFT size needs to be larger than configured number of samples inside chirp")
    val numZeros = fftSize - adcSamples
    //val cntInData  = RegInit(0.U(log2Up(params.maxFFTSize + 1 + CP_data_length).W))  // assume there is no CP inserted, so maxFFTSize is used as a size limitation


    val fields = Seq(
      // settable registers
      RegField(1, IQSwap,
        RegFieldDesc(name = "IQswap", desc = "I or Q data is first sent")),
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
      // add here optional fields
      RegField(log2fftSize, adcSamplesP1,
        RegFieldDesc(name = "adcSamplesP1", desc = "Number of samples per chirp for profile 1")),
      RegField(log2fftSize, adcSamplesP2,
        RegFieldDesc(name = "adcSamplesP2", desc = "Number of samples per chirp for profile 2")),
      RegField(log2fftSize, adcSamplesP3,
        RegFieldDesc(name = "adcSamplesP3", desc = "Number of samples per chirp for profile 3")),
      RegField(1, genLast,
        RegFieldDesc(name = "genLast", desc = "Used to enable or disable generation of last out signal")),
      RegField(8, chirpsPerFrame,
        RegFieldDesc(name = "chirpsPerFrame", desc = "Number of chirps per frame"))
    )

//        val pipe: Boolean = false,
//                        val flow: Boolean = false,
//                        val useSyncReadMem: Boolean = false,
//                        val useBlockRam: Boolean = false)


    // define abstract register map so it can be AXI4, Tilelink, APB, AHB
    regmap(fields.zipWithIndex.map({ case (f, i) => i * beatBytes -> Seq(f)}): _*)

    // end with registers

    for ((in, inIdx) <- ins.zipWithIndex) {
      val delayedInData = RegInit(0.U(16.W))
      val swap = RegInit(false.B)
      val delayedOutData = RegNext(outs(inIdx).bits.data, init = 0.U)

      val cntOutData = RegInit(0.U(log2fftSize.W))                   // assume there is no CP inserted, so maxFFTSize is used as a size limitation
      val cntChirps = RegInit(0.U(log2MaxChirpsPerFrame.W))
     //val zeroPaddFlag = RegInit(false.B)
      val zeroPaddFlag = Wire(Bool())

      in.ready := outs(inIdx).ready

      val dataQueue = Module(new QueueWithSyncReadMem(UInt((beatBytes*8).W), entries = params.queueSize, flow = false, useSyncReadMem = params.useBlockRam, useBlockRam = params.useBlockRam)) //Module(new Queue(UInt((beatBytes*8).W), entries = params.queueSize, flow = true)) //replace it with block ram! and check generation of last signal
      val outFire = outs(inIdx).valid && outs(inIdx).ready

      when (outFire) {
        cntOutData := cntOutData + 1.U
      }

      when (cntOutData === (fftSize - 1.U) && outFire) {
        cntChirps := cntChirps + 1.U
      }

      when (cntChirps === chirpsPerFrame) {
        cntChirps := 0.U
      }

      when (cntOutData === (fftSize - 1.U) && outFire) { //  check outs(inIdx).fire
        cntOutData := 0.U
        when (cntChirps === (chirpsPerFrame - 1.U)) {
          outs(inIdx).bits.last := true.B && genLast // active only one signal of clock
        }
        .otherwise {
          outs(inIdx).bits.last := false.B && genLast
        }
      }
    .otherwise {
        outs(inIdx).bits.last := false.B && genLast
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
      val inValidReg = RegNext(in.valid, init = false.B)
      val inDataReg = RegNext(in.bits.data, init = 0.U)


      when (testPattern || rawData) {
        // there is no need to do zero padding for those cases
        dataQueue.io.enq.bits   := in.bits.data // lsb bits are filled, by default msb bits are zero
        dataQueue.io.enq.valid  := in.valid
        dataQueue.io.deq.ready  := outs(inIdx).ready
      }
      .elsewhen (adcFormat === 0.U) {
        dataQueue.io.enq.bits  := inDataReg  // in.bits.data // lsb bits are filled, msb bits are zeros by default
        dataQueue.io.enq.valid := inValidReg // in.valid
        dataQueue.io.deq.ready := ~zeroPaddFlag
      // outs(inIdx).bits.data    := Cat(in.bits.data.asUInt, 0.U(16.W)) // lsb bits are filled, msb bits are zeros by default
      // outs(inIdx).valid        := in.valid
      }
      .otherwise {
      //.elsewhen (adcFormat === 1.U || adcFormat === 2.U) { // Complex 1-x
        when (inValidReg & ~swap) {
        //when (in.valid & ~swap) {
          delayedInData := inDataReg //in.bits.data
          swap := true.B
          dataQueue.io.enq.bits := 0.U
          dataQueue.io.enq.valid  := false.B
          // dataQueue.io.deq.ready := ~zeroPaddFlag // old
          dataQueue.io.deq.ready := (~zeroPaddFlag && outs(inIdx).ready) // new -> this outs(inIdx).ready highly important when fft is in the state sFlush
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
          dataQueue.io.deq.ready := (~zeroPaddFlag && outs(inIdx).ready) // new
        }
        .otherwise {
          // do not add new data, but load old if there are available data inside the Queue
          dataQueue.io.enq.bits  := delayedOutData
          dataQueue.io.enq.valid := false.B
          dataQueue.io.deq.ready := outs(inIdx).ready //false.B
        }
      }
      when (zeroPaddFlag) {
        outs(inIdx).valid := true.B
        outs(inIdx).bits.data := 0.U
        //in.ready := outs(inIdx).ready // can accept new data but that data is stored inside queue
        // but in theory this block should be always ready to accept data
        in.ready := dataQueue.io.enq.ready
      }
      .otherwise {
        outs(inIdx).valid := dataQueue.io.deq.valid
        outs(inIdx).bits.data := dataQueue.io.deq.bits
        in.ready := dataQueue.io.enq.ready
      }
    }
  }
}

class AXI4xWRdataPreProcMultipleStreams(address: AddressSet, params: AXI4XwrDataPreProcParams,  _beatBytes: Int = 4)(implicit p: Parameters) extends XWRdataPreProcMultipleStreamsBlock[AXI4MasterPortParameters, AXI4SlavePortParameters, AXI4EdgeParameters, AXI4EdgeParameters, AXI4Bundle](params, _beatBytes) with AXI4DspBlock with AXI4HasCSR {
  override val mem = Some(AXI4RegisterNode(address = address, beatBytes = _beatBytes))
}

class TLxWRdataPreProcMultipleStreams(address: AddressSet, params: AXI4XwrDataPreProcParams, _beatBytes: Int = 4)(implicit p: Parameters) extends XWRdataPreProcMultipleStreamsBlock[TLClientPortParameters, TLManagerPortParameters, TLEdgeOut, TLEdgeIn, TLBundle](params, _beatBytes) with TLDspBlock with TLHasCSR {
  val devname = "TLxWRdataPreProcMultipleStreams"
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

object AXI4xWRPreProcMultipleStreamsApp extends App
{
  val baseAddress = 0x500
  val params: AXI4XwrDataPreProcParams = AXI4XwrDataPreProcParams()

  implicit val p: Parameters = Parameters.empty

  val xWRDataPreProcModule = LazyModule(new AXI4xWRdataPreProcMultipleStreams(AddressSet(baseAddress, 0xFF), params, _beatBytes = 4) with AXI4xWRDataPreProcMultipleStreamsStandaloneBlock)
  chisel3.Driver.execute(args, ()=> xWRDataPreProcModule.module)  // change this to chisel stage
}
