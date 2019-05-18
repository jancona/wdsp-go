package wdsp

// #cgo LDFLAGS: -lwdsp
// #include <wdsp.h>
// #include <stdlib.h>
import "C"
import (
	"log"
	"unsafe"
)

// #define INREAL float
// #define OUTREAL float
// #define dINREAL float

// // WDSP vesion number
// int GetWDSPVersion();
func GetWDSPVersion() int {
	return int(C.GetWDSPVersion())
}

// // channel
// extern void OpenChannel (int channel, int in_size, int dsp_size, int input_samplerate, int dsp_rate, int output_samplerate, int type, int state, double tdelayup, double tslewup, double tdelaydown, double tslewdown, int bfo);
func OpenChannel(channel int, inSize int, dspSize int, inputSamplerate int, dspRate int, outputSamplerate int,
	typ int, state int, tdelayup float64, tslewup float64, tdelaydown float64, tslewdown float64, bfo int) {
	C.OpenChannel(C.int(channel), C.int(inSize), C.int(dspSize), C.int(inputSamplerate), C.int(dspRate), C.int(outputSamplerate),
		C.int(typ), C.int(state), C.double(tdelayup), C.double(tslewup), C.double(tdelaydown), C.double(tslewdown), C.int(bfo))
}

// extern void CloseChannel (int channel);
func CloseChannel(channel int) {
	C.CloseChannel(C.int(channel))
}

// extern void SetType (int channel, int type);
func SetType(channel int, typ int) {
	C.SetType(C.int(channel), C.int(typ))
}

// extern void SetInputBuffsize (int channel, int in_size);
func SetInputBuffsize(channel int, inSize int) {
	C.SetInputBuffsize(C.int(channel), C.int(inSize))
}

// extern void SetDSPBuffsize (int channel, int dsp_size);
func SetDSPBuffsize(channel int, dspSize int) {
	C.SetDSPBuffsize(C.int(channel), C.int(dspSize))
}

// extern void SetInputSamplerate  (int channel, int samplerate);
func SetInputSamplerate(channel int, samplerate int) {
	C.SetInputSamplerate(C.int(channel), C.int(samplerate))
}

// extern void SetDSPSamplerate (int channel, int samplerate);
func SetDSPSamplerate(channel int, samplerate int) {
	C.SetDSPSamplerate(C.int(channel), C.int(samplerate))
}

// extern void SetOutputSamplerate (int channel, int samplerate);
func SetOutputSamplerate(channel int, samplerate int) {
	C.SetOutputSamplerate(C.int(channel), C.int(samplerate))
}

// extern void SetAllRates (int channel, int in_rate, int dsp_rate, int out_rate);
func SetAllRates(channel int, inRate int, dspRate int, outRate int) {
	C.SetAllRates(C.int(channel), C.int(inRate), C.int(dspRate), C.int(outRate))
}

// int SetChannelState (int channel, int state, int dmode);
func SetChannelState(channel int, state int, dmode int) int {
	return int(C.SetChannelState(C.int(channel), C.int(state), C.int(dmode)))
}

// extern void SetChannelTDelayUp (int channel, double time);
func SetChannelTDelayUp(channel int, time float64) {
	C.SetChannelTDelayUp(C.int(channel), C.double(time))
}

// extern void SetChannelTSlewUp (int channel, double time);
func SetChannelTSlewUp(channel int, time float64) {
	C.SetChannelTSlewUp(C.int(channel), C.double(time))
}

// extern void SetChannelTDelayDown (int channel, double time);
func SetChannelTDelayDown(channel int, time float64) {
	C.SetChannelTDelayDown(C.int(channel), C.double(time))
}

// extern void SetChannelTSlewDown (int channel, double time);
func SetChannelTSlewDown(channel int, time float64) {
	C.SetChannelTSlewDown(C.int(channel), C.double(time))
}

// // iobuffs
// extern void fexchange0 (int channel, double* in, double* out, int* error);
func FExchange0(channel int, in []float64, out []float64, err *int) {
	errc := C.int(*err)
	C.fexchange0(C.int(channel), (*C.double)(unsafe.Pointer(&in[0])), (*C.double)(unsafe.Pointer(&out[0])), &errc)
	*err = int(errc)
}

// extern void fexchange2 (int channel, INREAL *Iin, INREAL *Qin, OUTREAL *Iout, OUTREAL *Qout, int* error);
func FExchange2(channel int, Iin []float32, Qin []float32, Iout []float32, Qout []float32, err *int) {
	errc := C.int(*err)
	C.fexchange2(C.int(channel), (*C.float)(unsafe.Pointer(&Iin[0])), (*C.float)(unsafe.Pointer(&Qin[0])), (*C.float)(unsafe.Pointer(&Iout[0])), (*C.float)(unsafe.Pointer(&Qout[0])), &errc)
	*err = int(errc)
}

// analyzer
const (
	detectorModePeak      = 0
	detectorModeRosenfell = 1
	detectorModeAverage   = 2
	detectorModeSample    = 3
)

const (
	averageModeNone         = 0
	averageModeRecursive    = 1
	averageModeTimeWindow   = 2
	averageModeLogRecursive = 3
)

// extern void XCreateAnalyzer(int disp, int *success, int m_size, int m_num_fft, int m_stitch, char *app_data_path);
func XCreateAnalyzer(dist int, success *int, mSize int, mNumFft int, mStitch int, appDataPath string) {
	log.Printf("XCreateAnalyzer(%d, %d, %d, %d, %d, %s)", dist, *success, mSize, mNumFft, mStitch, appDataPath)
	successC := C.int(*success)
	C.XCreateAnalyzer(C.int(dist), &successC, C.int(mSize), C.int(mNumFft), C.int(mStitch), nil)
	*success = int(successC)
}

// extern void SetAnalyzer(int disp, int n_pixout, int n_fft, int typ, int *flp, int sz, int bf_sz, int win_type, double pi, int ovrlp, int clp, int fscLin, int fscHin, int n_pix, int n_stch, int calset, double fmin, double fmax, int max_w);
func SetAnalyzer(dist int, nPixout int, nFft int, typ int, flp []int32, sz int, bfSz int, winType int, pi float64, ovrlp int, clp int, fscLin int, fscHin int, nPix int, nStch int, calset int, fmin float64, fmax float64, maxW int) {
	log.Printf("SetAnalyzer(%d, %d, %d, %d, %v, %d, %d, %d, %f, %d, %d, %d, %d, %d, %d, %d, %f, %f, %d)",
		dist, nPixout, nFft, typ, flp, sz, bfSz, winType, pi, ovrlp, clp, fscLin, fscHin, nPix, nStch, calset, fmin, fmax, maxW)
	C.SetAnalyzer(C.int(dist), C.int(nPixout), C.int(nFft), C.int(typ), (*C.int)(unsafe.Pointer(&flp[0])), C.int(sz), C.int(bfSz), C.int(winType), C.double(pi), C.int(ovrlp), C.int(clp), C.int(fscLin), C.int(fscHin), C.int(nPix), C.int(nStch), C.int(calset), C.double(fmin), C.double(fmax), C.int(maxW))
}

// extern void Spectrum0(int run, int disp, int ss, int LO, double* in);
func Spectrum0(run int, disp int, ss int, LO int, in []float64) {
	C.Spectrum0(C.int(run), C.int(disp), C.int(ss), C.int(LO), (*C.double)(unsafe.Pointer(&in[0])))
}

// extern void Spectrum(int disp, int ss, int LO, float* pI, float* pQ);
func Spectrum(disp int, ss int, LO int, I []float32, Q []float32) {
	C.Spectrum(C.int(disp), C.int(ss), C.int(LO), (*C.float)(unsafe.Pointer(&I[0])), (*C.float)(unsafe.Pointer(&Q[0])))
}

// extern void GetPixels(int disp, int pixout, float *pix, int *flag);
func GetPixels(disp int, pixout int, pix *[]float32, flag *int) {
	flagC := C.int(*flag)
	C.GetPixels(C.int(disp), C.int(pixout), (*C.float)(unsafe.Pointer(&(*pix)[0])), &flagC)
	*flag = int(flagC)
}

// extern void SetDisplayDetectorMode(int disp, int pixout, int mode);
func SetDisplayDetectorMode(disp int, pixout int, mode int) {
	C.SetDisplayDetectorMode(C.int(disp), C.int(pixout), C.int(mode))
}

// extern void SetDisplayAverageMode(int disp, int pixout, int mode);
func SetDisplayAverageMode(disp int, pixout int, mode int) {
	C.SetDisplayAverageMode(C.int(disp), C.int(pixout), C.int(mode))
}

// extern void SetDisplayNumAverage(int disp, int pixout, int num);
func SetDisplayNumAverage(disp int, pixout int, num int) {
	C.SetDisplayNumAverage(C.int(disp), C.int(pixout), C.int(num))
}

// extern void SetDisplayAvBackmult (int disp, int pixout, double mult);
func SetDisplayAvBackmult(disp int, pixout int, mult float64) {
	C.SetDisplayAvBackmult(C.int(disp), C.int(pixout), C.double(mult))
}

// extern void DestroyAnalyzer(int disp);
func DestroyAnalyzer(dist int) {
	C.DestroyAnalyzer(C.int(dist))
}

// RXA
// enum rxaMeterType {
//         RXA_S_PK,
//         RXA_S_AV,
//         RXA_ADC_PK,
//         RXA_ADC_AV,
//         RXA_AGC_GAIN,
//         RXA_AGC_PK,
//         RXA_AGC_AV,
//         RXA_METERTYPE_LAST
// };

const (
	RXA_S_PK = iota
	RXA_S_AV
	RXA_ADC_PK
	RXA_ADC_AV
	RXA_AGC_GAIN
	RXA_AGC_PK
	RXA_AGC_AV
	RXA_METERTYPE_LAST
)

// extern void SetRXAMode (int channel, int mode);
func SetRXAMode(channel int, mode int) {
	C.SetRXAMode(C.int(channel), C.int(mode))
}

// extern void SetRXABandpassRun (int channel, int run);
func SetRXABandpassRun(channel int, run int) {
	C.SetRXABandpassRun(C.int(channel), C.int(run))
}

// DEPRECATED extern void SetRXABandpassFreqs (int channel, double low, double high);
// extern void RXASetPassband (int channel, double f_low, double f_high);
func RXASetPassband(channel int, fLow float64, fHi float64) {
	C.RXASetPassband(C.int(channel), C.double(fLow), C.double(fHi))
}

// extern void SetRXAFMSQRun (int channel, int run);
func SetRXAFMSQRun(channel int, run int) {
	C.SetRXAFMSQRun(C.int(channel), C.int(run))
}

// extern void SetRXAFMSQThreshold (int channel, double threshold);
func SetRXAFMSQThreshold(channel int, threshold float64) {
	C.SetRXAFMSQThreshold(C.int(channel), C.double(threshold))
}

// extern void SetRXAAMSQRun (int channel, int run);
func SetRXAAMSQRun(channel int, run int) {
	C.SetRXAAMSQRun(C.int(channel), C.int(run))
}

// extern void SetRXAAMSQThreshold (int channel, double threshold);
func SetRXAAMSQThreshold(channel int, threshold float64) {
	C.SetRXAAMSQThreshold(C.int(channel), C.double(threshold))
}

// extern void SetRXAEMNRRun (int channel, int run);
func SetRXAEMNRRun(channel int, run int) {
	C.SetRXAEMNRRun(C.int(channel), C.int(run))
}

// extern void SetRXAEMNRgainMethod (int channel, int method);
func SetRXAEMNRgainMethod(channel int, method int) {
	C.SetRXAEMNRgainMethod(C.int(channel), C.int(method))
}

// extern void SetRXAEMNRnpeMethod (int channel, int method);
func SetRXAEMNRnpeMethod(channel int, method int) {
	C.SetRXAEMNRnpeMethod(C.int(channel), C.int(method))
}

// extern void SetRXAEMNRPosition (int channel, int position);
func SetRXAEMNRPosition(channel int, position int) {
	C.SetRXAEMNRPosition(C.int(channel), C.int(position))
}

// extern void SetRXAANFRun(int channel, int run);
func SetRXAANFRun(channel int, run int) {
	C.SetRXAANFRun(C.int(channel), C.int(run))
}

// extern double GetRXAMeter (int channel, int mt);
func GetRXAMeter(channel int, mt int) {
	C.GetRXAMeter(C.int(channel), C.int(mt))
}

// extern void SetRXAPanelBinaural(int channel, int bin);
func SetRXAPanelBinaural(channel int, bin int) {
	C.SetRXAPanelBinaural(C.int(channel), C.int(bin))
}

// extern void SetRXAPanelPan (int channel, double pan);
func SetRXAPanelPan(channel int, pan float64) {
	C.SetRXAPanelPan(C.int(channel), C.double(pan))
}

// DEPRECATED extern void RXANBPSetFreqs (int channel, double low, double high);
// DEPRECATED extern void SetRXASNBAOutputBandwidth (int channel, double low, double high);

// extern void SetRXAANRRun(int channel, int run);
func SetRXAANRRun(channel int, run int) {
	C.SetRXAANRRun(C.int(channel), C.int(run))
}

// extern void SetRXAEMNRaeRun (int channel, int run);
func SetRXAEMNRaeRun(channel int, run int) {
	C.SetRXAEMNRaeRun(C.int(channel), C.int(run))
}

// extern void SetRXASNBARun (int channel, int run);
func SetRXASNBARun(channel int, run int) {
	C.SetRXASNBARun(C.int(channel), C.int(run))
}

// extern void SetRXAANFRun(int channel, int run);
// func SetRXAANFRun(channel int, run int) {
// 	C.SetRXAANFRun(C.int(channel), C.int(run))
// }

// extern void SetRXAShiftRun (int channel, int run);
func SetRXAShiftRun(channel int, run int) {
	C.SetRXAShiftRun(C.int(channel), C.int(run))
}

// extern void SetRXAShiftFreq (int channel, double fshift);
func SetRXAShiftFreq(channel int, fshift float64) {
	C.SetRXAShiftFreq(C.int(channel), C.double(fshift))
}

// extern void SetRXAAMDSBMode(int channel, int sbmode);
func SetRXAAMDSBMode(channel int, sbmode int) {
	C.SetRXAAMDSBMode(C.int(channel), C.int(sbmode))
}

// extern void SetRXAANRVals (int channel, int taps, int delay, double gain, double leakage);
func SetRXAANRVals(channel int, taps int, delay int, gain float64, leakage float64) {
	C.SetRXAANRVals(C.int(channel), C.int(taps), C.int(delay), C.double(gain), C.double(leakage))
}

// extern void SetRXAAGCMode (int channel, int mode);
func SetRXAAGCMode(channel int, mode int) {
	C.SetRXAAGCMode(C.int(channel), C.int(mode))
}

// extern void SetRXAAGCFixed (int channel, double fixed_agc);
func SetRXAAGCFixed(channel int, fixedAGC float64) {
	C.SetRXAAGCFixed(C.int(channel), C.double(fixedAGC))
}

// extern void SetRXAAGCAttack (int channel, int attack);
func SetRXAAGCAttack(channel int, attack int) {
	C.SetRXAAGCAttack(C.int(channel), C.int(attack))
}

// extern void SetRXAAGCDecay (int channel, int decay);
func SetRXAAGCDecay(channel int, decay int) {
	C.SetRXAAGCDecay(C.int(channel), C.int(decay))
}

// extern void SetRXAAGCHang (int channel, int hang);
func SetRXAAGCHang(channel int, hang int) {
	C.SetRXAAGCHang(C.int(channel), C.int(hang))
}

// extern void GetRXAAGCHangLevel(int channel, double *hangLevel);
func GetRXAAGCHangLevel(channel int, hangLevel *float64) {
	hangLevelC := C.double(*hangLevel)
	C.GetRXAAGCHangLevel(C.int(channel), &hangLevelC)
	*hangLevel = float64(hangLevelC)
}

// extern void SetRXAAGCHangLevel(int channel, double hangLevel);
func SetRXAAGCHangLevel(channel int, hangLevel float64) {
	C.SetRXAAGCHangLevel(C.int(channel), C.double(hangLevel))
}

// extern void GetRXAAGCHangThreshold(int channel, int *hangthreshold);
func GetRXAAGCHangThreshold(channel int, hangThreshold *int) {
	hangThresholdC := C.int(*hangThreshold)
	C.GetRXAAGCHangThreshold(C.int(channel), &hangThresholdC)
	*hangThreshold = int(hangThresholdC)
}

// extern void SetRXAAGCHangThreshold (int channel, int hangthreshold);
func SetRXAAGCHangThreshold(channel int, hangThreshold int) {
	C.SetRXAAGCHangThreshold(C.int(channel), C.int(hangThreshold))
}

// extern void GetRXAAGCTop(int channel, double *max_agc);
func GetRXAAGCTop(channel int, maxAGC *float64) {
	maxAGCC := C.double(*maxAGC)
	C.GetRXAAGCTop(C.int(channel), &maxAGCC)
	*maxAGC = float64(maxAGCC)
}

// extern void SetRXAAGCTop (int channel, double max_agc);
func SetRXAAGCTop(channel int, maxAGC float64) {
	C.SetRXAAGCTop(C.int(channel), C.double(maxAGC))
}

// extern void SetRXAAGCSlope (int channel, int slope);
func SetRXAAGCSlope(channel int, slope int) {
	C.SetRXAAGCSlope(C.int(channel), C.int(slope))
}

// extern void SetRXAAGCThresh(int channel, double thresh, double size, double rate);
func SetRXAAGCThresh(channel int, thresh float64, size float64, rate float64) {
	C.SetRXAAGCThresh(C.int(channel), C.double(thresh), C.double(size), C.double(rate))
}

// extern void GetRXAAGCThresh(int channel, double *thresh, double size, double rate);
func GetRXAAGCThresh(channel int, thresh *float64, size float64, rate float64) {
	threshC := C.double(*thresh)
	C.GetRXAAGCThresh(C.int(channel), &threshC, C.double(size), C.double(rate))
	*thresh = float64(threshC)
}

// extern void SetRXAFMDeviation (int channel, double deviation);
func SetRXAFMDeviation(channel int, deviation float64) {
	C.SetRXAFMDeviation(C.int(channel), C.double(deviation))
}

// extern void RXASetNC(int channel, int nc);
func RXASetNC(channel int, nc int) {
	C.RXASetNC(C.int(channel), C.int(nc))
}

// extern void RXASetMP(int channel, int nc);
func RXASetMP(channel int, nc int) {
	C.RXASetMP(C.int(channel), C.int(nc))
}

// extern void SetRXAEQRun (int channel, int run);
func SetRXAEQRun(channel int, run int) {
	C.SetRXAEQRun(C.int(channel), C.int(run))
}

// DEPRECATED extern void SetRXAGrphEQ (int channel, int *rxeq);
// extern void RXANBPSetShiftFrequency (int channel, double shift);
func RXANBPSetShiftFrequency(channel int, shift float64) {
	C.RXANBPSetShiftFrequency(C.int(channel), C.double(shift))
}

// Diversity prototypes
// extern void create_divEXT (int id, int run, int nr, int size);
func Create_divEXT(id int, run int, nr int, size int) {
	C.create_divEXT(C.int(id), C.int(run), C.int(nr), C.int(size))
}

// extern void SetEXTDIVRun (int id, int run);
func SetEXTDIVRun(id int, run int) {
	C.SetEXTDIVRun(C.int(id), C.int(run))
}

// extern void SetEXTDIVBuffsize (int id, int size);
func SetEXTDIVBuffsize(id int, size int) {
	C.SetEXTDIVBuffsize(C.int(id), C.int(size))
}

// extern void SetEXTDIVNr (int id, int nr);
func SetEXTDIVNr(id int, nr int) {
	C.SetEXTDIVNr(C.int(id), C.int(nr))
}

// extern void SetEXTDIVOutput (int id, int output);
func SetEXTDIVOutput(id int, output int) {
	C.SetEXTDIVOutput(C.int(id), C.int(output))
}

// extern void SetEXTDIVRotate (int id, int nr, double *Irotate, double *Qrotate);
func SetEXTDIVRotate(id int, nr int, Irotate []float64, Qrotate []float64) {
	C.SetEXTDIVRotate(C.int(id), C.int(nr), (*C.double)(unsafe.Pointer(&Irotate[0])), (*C.double)(unsafe.Pointer(&Qrotate[0])))
}

// extern void xdivEXT (int id, int nsamples, double **in, double *out);

// Noise Blanker prototypes
// extern void create_anbEXT( int id, int run, int buffsize, double samplerate, double tau, double hangtime, double advtime, double backtau, double threshold);
func Create_anbEXT(
	id int,
	run int,
	buffsize int,
	samplerate float64,
	tau float64,
	hangtime float64,
	advtime float64,
	backtau float64,
	threshold float64,
) {
	C.create_anbEXT(
		C.int(id),
		C.int(run),
		C.int(buffsize),
		C.double(samplerate),
		C.double(tau),
		C.double(hangtime),
		C.double(advtime),
		C.double(backtau),
		C.double(threshold),
	)
}

// extern void destroy_anbEXT (int id);
func Destroy_anbEXT(
	id int,
) {
	C.destroy_anbEXT(
		C.int(id),
	)
}

// extern void flush_anbEXT (int id);
func Flush_anbEXT(
	id int,
) {
	C.flush_anbEXT(
		C.int(id),
	)
}

// extern void xanbEXT (int id, double* in, double* out);
func XanbEXT(
	id int,
	in *float64,
	out *float64,
) {
	inC := C.double(*in)
	outC := C.double(*out)
	C.xanbEXT(
		C.int(id),
		&inC,
		&outC,
	)
	*in = float64(inC)
	*out = float64(outC)
}

// extern void SetEXTANBRun (int id, int run);
// extern void SetEXTANBSamplerate (int id, int rate);
// extern void SetEXTANBTau (int id, double tau);
// extern void SetEXTANBHangtime (int id, double time);
// extern void SetEXTANBAdvtime (int id, double time);
// extern void SetEXTANBBacktau (int id, double tau);
// extern void SetEXTANBThreshold (int id, double thresh);
// extern void xanbEXTF (int id, float *I, float *Q);

// extern void create_nobEXT( int id, int run, int mode, int buffsize, double samplerate, double slewtime, double hangtime, double advtime, double backtau, double threshold);
func Create_nobEXT(
	id int,
	run int,
	mode int,
	buffsize int,
	samplerate float64,
	tau float64,
	hangtime float64,
	advtime float64,
	backtau float64,
	threshold float64,
) {
	C.create_nobEXT(
		C.int(id),
		C.int(run),
		C.int(mode),
		C.int(buffsize),
		C.double(samplerate),
		C.double(tau),
		C.double(hangtime),
		C.double(advtime),
		C.double(backtau),
		C.double(threshold),
	)
}

// extern void destroy_nobEXT (int id);
// extern void flush_nobEXT (int id);
// extern void xnobEXT (int id, double* in, double* out);
// extern void SetEXTNOBRun (int id, int run);
// extern void SetEXTNOBMode (int id, int mode);
// extern void SetEXTNOBBuffsize (int id, int size);
// extern void SetEXTNOBSamplerate (int id, int rate);
// extern void SetEXTNOBTau (int id, double tau);
// extern void SetEXTNOBHangtime (int id, double time);
// extern void SetEXTNOBAdvtime (int id, double time);
// extern void SetEXTNOBBacktau (int id, double tau);
// extern void SetEXTNOBThreshold (int id, double thresh);
// extern void xnobEXTF (int id, float *I, float *Q);

// // TXA Prototypes
// enum txaMeterType {
//   TXA_MIC_PK,
//   TXA_MIC_AV,
//   TXA_EQ_PK,
//   TXA_EQ_AV,
//   TXA_LVLR_PK,
//   TXA_LVLR_AV,
//   TXA_LVLR_GAIN,
//   TXA_CFC_PK,
//   TXA_CFC_AV,
//   TXA_CFC_GAIN,
//   TXA_COMP_PK,
//   TXA_COMP_AV,
//   TXA_ALC_PK,
//   TXA_ALC_AV,
//   TXA_ALC_GAIN,
//   TXA_OUT_PK,
//   TXA_OUT_AV,
//   TXA_METERTYPE_LAST
// };

// extern void SetTXAMode (int channel, int mode);
func SetTXAMode(
	channel int,
	mode int) {
	C.SetTXAMode(
		C.int(channel),
		C.int(mode),
	)
}

// extern void SetTXABandpassRun (int channel, int run);
// extern void SetTXABandpassFreqs (int channel, double low, double high);
// extern void SetTXABandpassWindow (int channel, int wintype);
// extern void SetTXAEQRun (int channel, int run);
// extern void SetTXACTCSSRun (int channel, int run);
// extern void SetTXAAMSQRun (int channel, int run);
// extern void SetTXACompressorGain (int channel, double gain);
// extern void SetTXACompressorRun (int channel, int run);
// extern void SetTXAosctrlRun (int channel, int run);
// extern void SetTXACFIRRun (int channel, int run);
// extern double GetTXAMeter (int channel, int mt);

// extern void SetTXAALCSt (int channel, int state);
// extern void SetTXAALCAttack (int channel, int attack);
// extern void SetTXAALCDecay (int channel, int decay);
// extern void SetTXAALCHang (int channel, int hang);

// extern void SetTXALevelerSt (int channel, int state);
// extern void SetTXALevelerAttack (int channel, int attack);
// extern void SetTXALevelerDecay (int channel, int decay);
// extern void SetTXALevelerHang (int channel, int hang);
// extern void SetTXALevelerTop (int channel, double maxgain);

// extern void SetTXAPreGenRun (int channel, int run);
// extern void SetTXAPreGenMode (int channel, int mode);
// extern void SetTXAPreGenToneMag (int channel, double mag);
// extern void SetTXAPreGenToneFreq (int channel, double freq);
// extern void SetTXAPreGenNoiseMag (int channel, double mag);
// extern void SetTXAPreGenSweepMag (int channel, double mag);
// extern void SetTXAPreGenSweepFreq (int channel, double freq1, double freq2);
// extern void SetTXAPreGenSweepRate (int channel, double rate);
// extern void SetTXAPreGenSawtoothMag (int channel, double mag);
// extern void SetTXAPreGenSawtoothFreq (int channel, double freq);
// extern void SetTXAPreGenTriangleMag (int channel, double mag);
// extern void SetTXAPreGenTriangleFreq (int channel, double freq);
// extern void SetTXAPreGenPulseMag (int channel, double mag);
// extern void SetTXAPreGenPulseFreq (int channel, double freq);
// extern void SetTXAPreGenPulseDutyCycle (int channel, double dc);
// extern void SetTXAPreGenPulseToneFreq (int channel, double freq);
// extern void SetTXAPreGenPulseTransition (int channel, double transtime);
// extern void SetTXAPostGenRun (int channel, int run);
// extern void SetTXAPostGenMode (int channel, int mode);
// extern void SetTXAPostGenToneMag (int channel, double mag);
// extern void SetTXAPostGenToneFreq (int channel, double freq);
// extern void SetTXAPostGenTTMag (int channel, double mag1, double mag2);
// extern void SetTXAPostGenTTFreq (int channel, double freq1, double freq2);
// extern void SetTXAPostGenSweepMag (int channel, double mag);
// extern void SetTXAPostGenSweepFreq (int channel, double freq1, double freq2);
// extern void SetTXAPostGenSweepRate (int channel, double rate);

// DEPRECATED extern void SetTXAGrphEQ (int channel, int *txeq);

// extern void SetTXAFMDeviation (int channel, double deviation);
// extern void SetTXAFMEmphPosition (int channel, int position);

// extern void TXASetNC(int channel, int nc);
// extern void TXASetMP(int channel, int nc);

// extern void SetTXAAMCarrierLevel (int channel, double c_level);

// // PureSignal
// extern void SetPSRunCal (int channel, int run);
// extern void SetPSMox (int channel, int mox);
// extern void SetPSReset (int channel, int reset);
// extern void SetPSMancal (int channel, int mancal);
// extern void SetPSAutomode (int channel, int automode);
// extern void SetPSTurnon (int channel, int turnon);
// extern void SetPSControl (int channel, int reset, int mancal, int automode, int turnon);
// extern void SetPSLoopDelay (int channel, double delay);
// extern void SetPSMoxDelay (int channel, double delay);
// extern double SetPSTXDelay (int channel, double delay);
// extern void SetPSHWPeak (int channel, double peak);
// extern void SetPSPtol (int channel, double ptol);
// extern void SetPSFeedbackRate (int channel, int rate);

// extern void GetPSInfo (int channel, int *info);
// extern void GetPSHWPeak (int channel, double* peak);
// extern void GetPSMaxTX (int channel, double* maxtx);

// extern void pscc (int channel, int size, double* tx, double* rx);

// // EER
// extern void create_eerEXT (int id, int run, int size, int rate, double mgain, double pgain, int rundelays, double mdelay, double pdelay, int amiq);
// extern void xeerEXTF (int id, float* inI, float* inQ, float* outI, float* outQ, float* outMI, float* outMQ, int mox, int size);
// extern void SetEERRun (int id, int run);
// extern void SetEERAMIQ (int id, int amiq);
// extern void SetEERRunDelays (int id, int run);
// extern void SetEERPgain (int id, double gain);
// extern void SetEERPdelay (int id, double delay);
// extern void SetEERMgain (int id, double gain);
// extern void SetEERMdelay (int id, double delay);

// // resampler

// extern void *create_resample (int run, int size, double* in, double* out, int in_rate, int out_rate, double fc, int ncoef, double gain);
// extern void destroy_resample (void *a);
// extern void flush_resample (void *a);
// extern int xresample (void *a);

// extern void* create_resampleFV (int in_rate, int out_rate);
// extern void xresampleFV (float* input, float* output, int numsamps, int* outsamps, void* ptr);
// extern void destroy_resampleFV (void* ptr);

// // patchpanel

// extern void SetRXAPanelRun (int channel, int run);
// extern void SetRXAPanelSelect (int channel, int select);
// extern void SetRXAPanelGain1 (int channel, double gain);
// extern void SetRXAPanelGain2 (int channel, double gainI, double gainQ);
// extern void SetRXAPanelPan (int channel, double pan);
// extern void SetRXAPanelCopy (int channel, int copy);
// extern void SetRXAPanelBinaural (int channel, int bin);

// extern void SetTXAPanelRun (int channel, int run);
// extern void SetTXAPanelGain1 (int channel, double gain);

// // variable smapler

// extern void* create_varsampV (int in_rate, int out_rate, int R);
// extern void xvarsampV (double* input, double* output, int numsamps, double var, int* outsamps, void* ptr);
// extern void destroy_varsampV (void* ptr);

// // CTSCC
// extern void SetTXACTCSSFreq (int channel, double freq);

// // wisdom
// char *wisdom_get_status();
// extern void WDSPwisdom (char* directory);

func WDSPwisdom(directory string) {
	dir := C.CString(directory)
	defer C.free(unsafe.Pointer(dir))
	C.WDSPwisdom(dir)
}
