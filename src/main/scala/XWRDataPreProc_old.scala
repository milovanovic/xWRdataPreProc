// package xWRDataPreProc
// 
// import chisel3._
// import chisel3.util._
// import chisel3.experimental._
// 
// import dsptools._
// import dsptools.numbers._
// 
// import dspblocks._
// import freechips.rocketchip.amba.axi4._
// import freechips.rocketchip.amba.axi4stream._
// import freechips.rocketchip.config._
// import freechips.rocketchip.diplomacy._
// import freechips.rocketchip.regmapper._
// import freechips.rocketchip.tilelink._
// 
// 
// trait AXI4xWRDataPreProcStandaloneBlock extends AXI4xWRdataPreProcBlock {
//   def standaloneParams = AXI4BundleParameters(addrBits = 32, dataBits = 32, idBits = 1)
//   val ioMem = mem.map { m => {
//     val ioMemNode = BundleBridgeSource(() => AXI4Bundle(standaloneParams))
// 
//     m :=
//       BundleBridgeToAXI4(AXI4MasterPortParameters(Seq(AXI4MasterParameters("bundleBridgeToAXI4")))) :=
//       ioMemNode
// 
//     val ioMem = InModuleBody { ioMemNode.makeIO() }
//     ioMem
//   }}
//   
//   val ioInNode = BundleBridgeSource(() => new AXI4StreamBundle(AXI4StreamBundleParameters(n = 2)))
//   val ioOutNode = BundleBridgeSink[AXI4StreamBundle]()
// 
//   ioOutNode :=
//     AXI4StreamToBundleBridge(AXI4StreamSlaveParameters()) :=
//     streamNode :=
//     BundleBridgeToAXI4Stream(AXI4StreamMasterParameters(n = 4)) :=
//     ioInNode
// 
//   val in = InModuleBody { ioInNode.makeIO() }
//   val out = InModuleBody { ioOutNode.makeIO() }
// }
// 
// 
// abstract class XWRdataPreProcBlock [ D, U, E, O, B <: Data] (beatBytes: Int, mergeInRawMode: Boolean) extends LazyModule()(Parameters.empty) with DspBlock[D, U, E, O, B] with HasCSR {
//   // This core should also generate lastOut signal depeneding on number of frames, number of chirps etc.
//   
//   val masterParams = AXI4StreamMasterParameters(
//     name = "AXI4 Stream xWrDataPreProc",
//     n = 4, // 4*8 -> 32 bits
//     numMasters = 1
//   )
//   val slaveParams = AXI4StreamSlaveParameters()
//   
//   val slaveNode  = AXI4StreamSlaveNode(slaveParams)
//   val masterNode = AXI4StreamMasterNode(masterParams)
//   
//   val streamNode = NodeHandle(slaveNode, masterNode)
//   
//   lazy val module = new LazyModuleImp(this) {
// //     val (in, _)  = streamNode.in(0)
// //     val (out, _) = streamNode.out(0)
//     val (out, edgeOut) = masterNode.out.head
//     val (in, edgeIn) = slaveNode.in.head
// 
//     // Define Control registers
//     val IQSwap = RegInit(false.B)        // default is 0, I first
//     val adcFormat = RegInit(0.U(2.W))    // default is 0, Real
//     val testPattern = RegInit(true.B)    // default is 1, Test Pattern - change this in the future
//     val rawData = RegInit(true.B)        // rawData is on, dsp part is off
//     // val msbFirst = RegInit(true)      // default is 1, msb first is 
//     // val crcEnable = RegInit(false.B)  // default is off, crcEnable is off
//     val delayedInData = RegInit(0.U(16.W))
//     val swap = RegInit(true.B)
//     val delayedOutData = RegNext(out.bits.data)
//     // Status registers
//     // val CRCerror        = RegInit(false.B)
// 
//     val fields = Seq(
//       // settable registers
//       RegField(1, IQSwap,
//         RegFieldDesc(name = "IQ swap", desc = "I or Q data is first sent")),
//       RegField(2, adcFormat,
//         RegFieldDesc(name = "adcFormat", desc = "")),
//       RegField(1, testPattern,
//         RegFieldDesc(name = "testPattern", desc = "")),
//       RegField(1, rawData, 
//         RegFieldDesc(name = "rawData", desc = ""))
//       // read-only status registers
//       //RegField.r(1, msbFirst,
//         //RegFieldDesc(name = "msbFirst", desc = ""))
//     )
// 
//     
//     
//     // Define abstract register map so it can be AXI4, Tilelink, APB, AHB
//     regmap(fields.zipWithIndex.map({ case (f, i) => i * beatBytes -> Seq(f)}): _*)
//     
//     in.ready := out.ready
//     
//     if (mergeInRawMode) {
//       when (testPattern || rawData) {
//         when (in.valid & ~swap) {
//           delayedInData := in.bits.data
//           swap := true.B
//           out.bits.data := 0.U
//           out.valid := false.B
//         }
//         .elsewhen (in.valid & swap) {
//           swap := false.B
//           out.valid := true.B
//           out.bits.data := Cat(delayedInData.asUInt, in.bits.data.asUInt)
//         }
//         .otherwise {
//           out.bits.data := delayedOutData
//           out.valid := false.B
//         }
//       }
//       .elsewhen (adcFormat === 0.U) {
//         out.bits.data    := in.bits.data // lsb bits are filled, by default msb bits are zero
//         out.valid        := in.valid
//       }
//       .otherwise {
//         when (in.valid & ~swap) {
//           delayedInData := in.bits.data
//           swap := true.B
//           out.bits.data := 0.U
//           out.valid := false.B
//         }
//         .elsewhen (in.valid & swap) {
//           swap := false.B
//           out.valid := true.B
//           when (IQSwap) {
//             out.bits.data := Cat(delayedInData.asUInt, in.bits.data.asUInt)
//           }
//           .otherwise {
//             out.bits.data := Cat(in.bits.data.asUInt, delayedInData.asUInt)
//           }
//         }
//         .otherwise {
//           out.bits.data := delayedOutData
//           out.valid := false.B
//         }
//       }
//     }
//     else {
//       when (testPattern || rawData || adcFormat === 0.U) {
//         out.bits.data    := in.bits.data // lsb bits are filled, by default msb bits are zero 
//         out.valid        := in.valid
//       }
//       .elsewhen (adcFormat === 1.U) { // Complex 1-x
//         when (in.valid & ~swap) {
//           delayedInData := in.bits.data
//           swap := true.B
//           out.bits.data := 0.U
//           out.valid := false.B
//         }
//         .elsewhen (in.valid & swap) {
//           swap := false.B
//           out.valid := true.B
//           when (IQSwap) {
//             out.bits.data := Cat(delayedInData.asUInt, in.bits.data.asUInt)
//           }
//           .otherwise {
//             out.bits.data := Cat(in.bits.data.asUInt, delayedInData.asUInt)
//           }
//         }
//         .otherwise {
//           out.bits.data := delayedOutData
//           out.valid := false.B
//         }
//       }
//     }
//   }
// }
// 
// class AXI4xWRdataPreProcBlock(address: AddressSet, _beatBytes: Int = 4)(implicit p: Parameters) extends XWRdataPreProcBlock[AXI4MasterPortParameters, AXI4SlavePortParameters, AXI4EdgeParameters, AXI4EdgeParameters, AXI4Bundle](_beatBytes) with AXI4DspBlock with AXI4HasCSR {
//   override val mem = Some(AXI4RegisterNode(address = address, beatBytes = _beatBytes))
// }
// 
// class TLxWRdataPreProcBlock(address: AddressSet, _beatBytes: Int = 4)(implicit p: Parameters) extends XWRdataPreProcBlock[TLClientPortParameters, TLManagerPortParameters, TLEdgeOut, TLEdgeIn, TLBundle](_beatBytes) with TLDspBlock with TLHasCSR {
//   val devname = "TLxWRdataPreProcBlock"
//   val devcompat = Seq("xWRDataPreProc", "radardsp")
//   val device = new SimpleDevice(devname, devcompat) {
//     override def describe(resources: ResourceBindings): Description = {
//       val Description(name, mapping) = super.describe(resources)
//       Description(name, mapping)
//     }
//   }
//   // make diplomatic TL node for regmap
//   override val mem = Some(TLRegisterNode(address = Seq(address), device = device, beatBytes = _beatBytes))
// }
// 
// 
// object AXI4xWRPreProcBlockApp extends App
// {
//   val baseAddress = 0x500 // just to check if verilog code is succesfully generated or not
//   implicit val p: Parameters = Parameters.empty
// 
//   val xWRDataPreProcModule = LazyModule(new AXI4xWRdataPreProcBlock(AddressSet(baseAddress, 0xFF), _beatBytes = 4) with AXI4xWRDataPreProcStandaloneBlock)
//   chisel3.Driver.execute(args, ()=> xWRDataPreProcModule.module)
// }
