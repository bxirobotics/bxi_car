// Timing symbols for board/runtime tests. The mock vendor never initializes
// timing; a test that enables CAN6 must reject missing timing verification.
extern "C" int XCanFd_SetBaudRatePrescaler(void *, unsigned char) {return -1;}
extern "C" unsigned char XCanFd_GetBaudRatePrescaler(void *) {return 0;}
extern "C" int XCanFd_SetBitTiming(void *, unsigned char, unsigned char, unsigned short)
{
  return -1;
}
extern "C" void XCanFd_GetBitTiming(
  void *, unsigned char * sjw, unsigned char * tseg2, unsigned char * tseg1)
{
  *sjw = *tseg2 = *tseg1 = 0;
}
// test_bxi_runtime compiles board/timing directly, without linker wrapping.
extern "C" int __real_XCanFd_SetBaudRatePrescaler(void *, unsigned char) {return -1;}
extern "C" int __real_XCanFd_SetBitTiming(void *, unsigned char, unsigned char, unsigned short)
{
  return -1;
}
