package boom.v3.perf

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v3.common._

// For TAPrefetcher
class LSUTraceRecord(implicit p: Parameters) extends BoomBundle {
  val pc = UInt(xLen.W)
  val addr = UInt(xLen.W)
  val time = UInt(xLen.W)
  val tpe = UInt(4.W)
  val isLoad = Bool()
  val isIncomingLoad = Bool()
  val prv = UInt(2.W)
  val rob_idx = UInt(robAddrSz.W)
}

class LSUTraceIO(implicit p: Parameters) extends BoomBundle {
  val records = Output(Vec(memWidth, Valid(new LSUTraceRecord())))
}

// RoCC modules implemented outside Boom can expose these interfaces without
// making Boom depend on the prefetcher implementation.
trait HasLSUTrace {
  def lsuTrace: LSUTraceIO
}

