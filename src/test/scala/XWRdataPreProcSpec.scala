package xWRDataPreProc

import chisel3._
import chisel3.experimental._
import chisel3.util._

import chisel3.iotesters.Driver
import chisel3.iotesters.PeekPokeTester
import dspblocks.{AXI4DspBlock, AXI4StandaloneBlock}

import freechips.rocketchip.amba.axi4stream._
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.diplomacy._

import breeze.math.Complex
import breeze.signal.{fourierTr, iFourierTr}
import breeze.linalg._
import org.scalatest.{FlatSpec, Matchers}
import scala.math.{Pi, pow}


class XWRdataPreProcTester
(
  dut: AXI4xWRdataPreProcBlock with AXI4xWRDataPreProcStandaloneBlock,
  csrAddress: AddressSet,
  beatBytes : Int = 4,
  silentFail: Boolean = false
) extends PeekPokeTester(dut.module) with AXI4StreamModel with AXI4MasterModel {
  //override def memAXI: AXI4Bundle = dut.ioMem.get.getWrappedValue
  def memAXI: AXI4Bundle = dut.ioMem.get

  val mod = dut.module
  val fftSize = 32
  val numSamples = 24
  
  val master = bindMaster(dut.in.getWrappedValue)//bindMaster(dut.in)
  
  // generate simple test array!
  val inData = 1 to numSamples toArray
  
  //println(inData.length.toString)
  //println("Input data is:")
  //inData.map(c => println(c.toString))

  step(1)
  poke(dut.out.ready, true.B)
  var cycle = 0
  
  // test raw real data
  memWriteWord(csrAddress.base + 1*beatBytes, BigInt(0)) // real
  // test complex raw data
  //memWriteWord(csrAddress.base + 1*beatBytes, BigInt(1)) // complex
  memWriteWord(csrAddress.base + 2*beatBytes, BigInt(0))   // turn of testPattern mode
  memWriteWord(csrAddress.base + 3*beatBytes, BigInt(0))   // turn of rawData mode
  memWriteWord(csrAddress.base + 5*beatBytes, BigInt(numSamples))
  memWriteWord(csrAddress.base + 4*beatBytes, BigInt(fftSize))
  
  master.addTransactions((0 until inData.size).map(i => AXI4StreamTransaction(data = inData(i))))
  master.addTransactions((0 until inData.size).map(i => AXI4StreamTransaction(data = inData(i))))

  master.addTransactions((0 until inData.size).map(i => AXI4StreamTransaction(data = inData(i)))) 
  master.addTransactions((0 until inData.size).map(i => AXI4StreamTransaction(data = inData(i))))

  
  var outSeq = Seq[Int]()
  var peekedVal: BigInt = 0
  var realSeq = Seq[Int]()
  var imagSeq = Seq[Int]()
  var tmpReal: Short = 0
  var tmpImag: Short = 0

  // check only one fft window 
  while (outSeq.length < fftSize) {
    if (peek(dut.out.valid) == 1 && peek(dut.out.ready) == 1) {
      peekedVal = peek(dut.out.bits.data)
      outSeq = outSeq :+ peekedVal.toInt
      tmpReal = (peekedVal.toInt / pow(2,16)).toShort
      tmpImag = (peekedVal.toInt - (tmpReal.toInt * pow(2,16))).toShort
      realSeq = realSeq :+ tmpReal.toInt
      imagSeq = imagSeq :+ tmpImag.toInt
      println(peekedVal.toInt.toString)
    }
    step(1)
  }
  
  step(100)
  stepToCompletion(100, silentFail = silentFail)
}

class XWRdataPreProcSpec extends FlatSpec with Matchers {
  implicit val p: Parameters = Parameters.empty
  val params: AXI4XwrDataPreProcParams = AXI4XwrDataPreProcParams()

  it should "Test XWR data preproc block" in {
    val lazyDut = LazyModule(new AXI4xWRdataPreProcBlock(address = AddressSet(0x010000, 0xFFF), params = params, _beatBytes = 4) with AXI4xWRDataPreProcStandaloneBlock)
    
    chisel3.iotesters.Driver.execute(Array("-tiwv", "-tbn", "verilator", "-tivsuv"), () => lazyDut.module) {
      c => new XWRdataPreProcTester(lazyDut, csrAddress = AddressSet(0x010000, 0xFFF), beatBytes = 4, silentFail = true)
    } should be (true)
  }
}
