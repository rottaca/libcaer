/**
 * @file dynapse.h
 *
 * Dynap-se specific configuration defines and information structures.
 */

#ifndef LIBCAER_DEVICES_DYNAPSE_H_
#define LIBCAER_DEVICES_DYNAPSE_H_

#include "usb.h"
#include "../events/spike.h"
#include "../events/special.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Device type definition for iniLabs Dynap-se FX2-based boards.
 */
#define CAER_DEVICE_DYNAPSE 3

/**
 * Dynap-se chip identifier.
 */
#define DYNAPSE_CHIP_DYNAPSE 64

/**
 * Module address: device-side Multiplexer configuration.
 * The Multiplexer is responsible for mixing, timestamping and outputting
 * (via USB) the various event types generated by the device. It is also
 * responsible for timestamp generation.
 */
#define DYNAPSE_CONFIG_MUX      0
/**
 * Module address: device-side AER configuration (from chip).
 * The AER state machine handshakes with the chip's AER bus and gets the
 * spike events from it. It supports various configurable delays.
 */
#define DYNAPSE_CONFIG_AER      1
/**
 * Module address: device-side chip control configuration.
 * This state machine is responsible for configuring the chip's internal
 * control registers, to set special options and biases.
 */
#define DYNAPSE_CONFIG_CHIP     5
/**
 * Module address: device-side system information.
 * The system information module provides various details on the device, such
 * as currently installed logic revision or clock speeds.
 * All its parameters are read-only.
 * This is reserved for internal use and should not be used by
 * anything other than libcaer. Please see the 'struct caer_dynapse_info'
 * documentation for more details on what information is available.
 */
#define DYNAPSE_CONFIG_SYSINFO  6
/**
 * Module address: device-side USB output configuration.
 * The USB output module forwards the data from the device and the FPGA/CPLD to
 * the USB chip, usually a Cypress FX2 or FX3.
 */
#define DYNAPSE_CONFIG_USB      9
/**
 * Clear CAM content
 * Output USB data packets in streams of 512 bytes using libusb
 * es: caerConfigSet(moduleData->moduleState, DYNAPSE_CONFIG_CLEAR_CAM, 0, 0); //0,0 not used
 */
#define DYNAPSE_CONFIG_CLEAR_CAM 10
/**
 * Clear SRAM content, use one SRAM cell to monitor neurons
 * Output USB data packets in streams of 512 bytes using libusb
 * es: caerConfigSet(moduleData->moduleState, DYNAPSE_CONFIG_DEFAULT_SRAM, DYNAPSE_CONFIG_DYNAPSE_U2, 0); // zero not used
 */
#define DYNAPSE_CONFIG_DEFAULT_SRAM 11
/**
 * Used to monitor neurons , example usage:
 * es: caerConfigSet(moduleData->moduleState, DYNAPSE_CONFIG_MONITOR_NEU, 1, 0);  // core 1 neuron 0
 *
 * */
#define DYNAPSE_CONFIG_MONITOR_NEU 12
/**
 * Clear SRAM content, route nothing outside
 * Output USB data packets in streams of 512 bytes using libusb
 * es: caerConfigSet(moduleData->moduleState, DYNAPSE_CONFIG_DEFAULT_SRAM, DYNAPSE_CONFIG_DYNAPSE_U2, 0); // zero not used
 */
#define DYNAPSE_CONFIG_DEFAULT_SRAM_EMPTY 13

/**
 * Module address: device side SRAM controller configuration.
 * The module holds an address, a word to be written to SRAM
 * the most recent word read using a read command, and a read/write command.
 * Reads/writes are triggered when the address field is changed
 * ex: caerDynapseWriteSramWords(moduleData->moduleState, SRAMData, baseAddr, numWords);
 * Writes numWords words from array SRAMData to the SRAM, starting at baseAddr.
 */

#define DYNAPSE_CONFIG_SRAM 14

/**
 * Module address: Device side Synapse Reconfiguration module configuration.
 * Provides run control, selection between using a single kernel for
 * all neurons and reading per-neuron kernels from SRAM, programming of the
 * global kernel, as well as target output chip ID selection and SRAM kernel
 * table base address.
 */
#define DYNAPSE_CONFIG_SYNAPSERECONFIG 15

/**
 * Module address: Device side spike generator module configuration.
 * Provides start/stop control of spike train application and selection
 * of fixed/variable interspike intervals and their location in memory.
 */
#define DYNAPSE_CONFIG_SPIKEGEN 16

/**
 * Parameter address for module DYNAPSE_CONFIG_SPIKEGEN.
 * Instructs the spike generator to start applying the configurated
 * spike train when the parameter changes from false to true.
 */
#define DYNAPSE_CONFIG_SPIKEGEN_RUN 0

/**
 * Parameter address for module DYNAPSE_CONFIG_SPIKEGEN.
 * Selects variable interspike interval mode (true) or fixed interspike
 * interval (false).
 */
#define DYNAPSE_CONFIG_SPIKEGEN_VARMODE 1

/**
 * Parameter address for module DYNAPSE_CONFIG_SPIKEGEN.
 * Sets the start address of a spike train in memory.
 */
#define DYNAPSE_CONFIG_SPIKEGEN_BASEADDR 2

/**
 * Paramter address for module DYNAPSE_CONFIG_SPIKEGEN.
 * Sets the number of events to read from memory for a single application
 * of a spike train.
 */
#define DYNAPSE_CONFIG_SPIKEGEN_STIMCOUNT 3

/**
 * Parameter address for module DYNAPSE_CONFIG_SPIKEGEN.
 * Sets the interspike interval that will be used in fixed ISI mode (VARMODE false).
 */
#define DYNAPSE_CONFIG_SPIKEGEN_ISI 4

/**
 * Parameter address for module DYNAPSE_CONFIG_SPIKEGEN.
 * Sets the time base resolution for interspike intervals as the number
 * of FPGA clock cycles.
 */
#define DYNAPSE_CONFIG_SPIKEGEN_ISIBASE 5

/**
 * Parameter address for module DYNAPSE_CONFIG_SYNAPSERECONFIG:
 * Run control. Starts and stops handshaking with DVS.
 */
#define DYNAPSE_CONFIG_SYNAPSERECONFIG_RUN 0

/**
 * Parameter address for module DYNAPSE_CONFIG_SYNAPSERECONFIG
 * Bits 16 down to 12 select the address in the global kernel table
 * and bits 11 down to 0 specify the data.
 * The 12 data bits are split into 4*3 synaptic weight bits which map
 * onto positive/negative polarity events from 2 DVS pixels.
 */
#define DYNAPSE_CONFIG_SYNAPSERECONFIG_GLOBALKERNEL 1

/**
 * Parameter address for module DYNAPSE_CONFIG_SYNAPSERECONFIG
 * Boolean parameter for selecting between using kernels stored in
 * SRAM or the global kernel table. 1 for SRAM, 0 for global kernel table
 */
#define DYNAPSE_CONFIG_SYNAPSERECONFIG_USESRAMKERNELS 2

/**
 * Parameter address for moudle DYNAPSE_CONFIG_SYNAPSERECONFIG
 * Output chip select using chip identifiers from this document
 */
#define DYNAPSE_CONFIG_SYNAPSERECONFIG_CHIPSELECT 3

/**
 * Parameter address for module DYNAPSE_CONFIG_SYNAPSERECONFIG
 * SRAM base address configuration in increments of 32 Kib.
 * Setting this to N will place the SRAM kernel LUT in the range [N*2^15,(N+1)*2^15-1]
 */
#define DYNAPSE_CONFIG_SYNAPSERECONFIG_SRAMBASEADDR 4

/**
 * Parameter address for module DYNAPSE_CONFIG_SRAM:
 * Holds the address that will be used for the next read/write.
 * Writing or reading this field will trigger the command contained
 * in the command register to be executed.
 */
#define DYNAPSE_CONFIG_SRAM_ADDRESS 1

/**
 *Parameter address for module DYNAPSE_CONFIG_SRAM:
 * Holds the most recently read data from the SRAM.
 * Read only parameter.
 */
#define DYNAPSE_CONFIG_SRAM_READDATA 2

/**
 * Parameter address for module DYNAPSE_CONFIG_SRAM:
 * Holds the data that will be written on the next write.
 * ex: caerConfigSet(moduleData->moduleState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_WRITEDATA, wData);
 *     caerConfigSet(moduleData->moduleState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_RWCOMMAND, DYNAPSE_CONFIG_SRAM_WRITE);
 *     caerConfigSet(moduleData->moduleState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_ADDRESS, wAddr);
 * Writes wData to the address specified by wAddr.
 */
#define DYNAPSE_CONFIG_SRAM_WRITEDATA 3

/**
 * Parameter address for module DYNAPSE_CONFIG_SRAM:
 * Holds the command that will be executed when the address field is written to.
 * ex: caerConfigSet(moduleData->moduleState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_RWCOMMAND, DYNAPSE_CONFIG_SRAM_WRITE);
 * Sets the SRAM controller up for doing writes.
 */
#define DYNAPSE_CONFIG_SRAM_RWCOMMAND 4

/**
 * Command for module DYNAPSE_CONFIG_SRAM:
 * Write command for the RWCOMMAND field.
 * ex: caerConfigSet(moduleData->moduleState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_RWCOMMAND, DYNAPSE_CONFIG_SRAM_WRITE);
 * Sets the SRAM controller up for doing writes.
 */
#define DYNAPSE_CONFIG_SRAM_WRITE 1

/**
 * Command for module DYNAPSE_CONFIG_SRAM:
 * Read command for the RWCOMMAND field.
 * ex: caerConfigSet(moduleData->moduleState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_RWCOMMAND, DYNAPSE_CONFIG_SRAM_READ);
 * Sets the SRAM controller up for doing reads.
 */
#define DYNAPSE_CONFIG_SRAM_READ 0

/**
 * Parameter address for module DYNAPSE_CONFIG_SRAM:
 * Burst mode enable for fast writing. Disables updates on address change and instead updates on data change,
 * while automatically incrementing the writing address. Two 16-bit words are written per 32-bit word sent
 * to the SPI controller starting with the least significant half word.
 */
#define DYNAPSE_CONFIG_SRAM_BURSTMODE 5

/**
 * Parameter address for module DYNAPSE_CONFIG_MUX:
 * run the Multiplexer state machine, which is responsible for
 * mixing the various event types at the device level, timestamping
 * them and outputting them via USB or other connectors.
 */
#define DYNAPSE_CONFIG_MUX_RUN                             0
/**
 * Parameter address for module DYNAPSE_CONFIG_MUX:
 * run the Timestamp Generator inside the Multiplexer state machine,
 * which will provide microsecond accurate timestamps to the
 * events passing through.
 */
#define DYNAPSE_CONFIG_MUX_TIMESTAMP_RUN                   1
/**
 * Parameter address for module DYNAPSE_CONFIG_MUX:
 * reset the Timestamp Generator to zero. This also sends a reset
 * pulse to all connected slave devices, resetting their timestamp too.
 */
#define DYNAPSE_CONFIG_MUX_TIMESTAMP_RESET                 2
/**
 * Parameter address for module DYNAPSE_CONFIG_MUX:
 * under normal circumstances, the chip's bias generator is only powered
 * up when either the AER or the configuration state machines are running, to save
 * power. This flag forces the bias generator to be powered up all the time.
 */
#define DYNAPSE_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE          3
/**
 * Parameter address for module DYNAPSE_CONFIG_MUX:
 * drop AER events if the USB output FIFO is full, instead of having
 * them pile up at the input FIFOs.
 */
#define DYNAPSE_CONFIG_MUX_DROP_AER_ON_TRANSFER_STALL      4

/**
 * Parameter address for module DYNAPSE_CONFIG_AER:
 * run the AER state machine and get spike events from the chip by
 * handshaking with its AER bus.
 */
#define DYNAPSE_CONFIG_AER_RUN                    3
/**
 * Parameter address for module DYNAPSE_CONFIG_AER:
 * delay capturing the data and acknowledging it on the AER bus for
 * the events by this many LogicClock cycles.
 */
#define DYNAPSE_CONFIG_AER_ACK_DELAY          4
/**
 * Parameter address for module DYNAPSE_CONFIG_AER:
 * extend the length of the acknowledge on the AER bus for
 * the events by this many LogicClock cycles.
 */
#define DYNAPSE_CONFIG_AER_ACK_EXTENSION      6
/**
 * Parameter address for module DYNAPSE_CONFIG_AER:
 * if the output FIFO for this module is full, stall the AER handshake with
 * the chip and wait until it's free again, instead of just continuing
 * the handshake and dropping the resulting events.
 */
#define DYNAPSE_CONFIG_AER_WAIT_ON_TRANSFER_STALL 8
/**
 * Parameter address for module DYNAPSE_CONFIG_AER:
 * enable external AER control. This ensures the chip and the neuron
 * array are running, but doesn't do the handshake and leaves the ACK
 * pin in high-impedance, to allow for an external system to take
 * over the AER communication with the chip.
 * DYNAPSE_CONFIG_AER_RUN has to be turned off for this to work.
 */
#define DYNAPSE_CONFIG_AER_EXTERNAL_AER_CONTROL   10

/**
 * Parameter address for module DYNAPSE_CONFIG_CHIP:
 * enable the configuration AER state machine to send
 * bias and control configuration to the chip.
 */
#define DYNAPSE_CONFIG_CHIP_RUN             0
/**
 * Parameter address for module DYNAPSE_CONFIG_CHIP:
 * set the chip ID to which configuration content is
 * being sent.
 */
#define DYNAPSE_CONFIG_CHIP_ID              1
/**
 * Parameter address for module DYNAPSE_CONFIG_CHIP:
 * set the configuration content to send to the chip.
 * Every time this changes, the chip ID is appended
 * and the configuration is sent out to the chip.
 */
#define DYNAPSE_CONFIG_CHIP_CONTENT         2
/**
 * Parameter address for module DYNAPSE_CONFIG_CHIP:
 * delay doing the request after putting out the data
 * by this many LogicClock cycles.
 */
#define DYNAPSE_CONFIG_CHIP_REQ_DELAY       3
/**
 * Parameter address for module DYNAPSE_CONFIG_CHIP:
 * extend the request after receiving the ACK by
 * this many LogicClock cycles.
 */
#define DYNAPSE_CONFIG_CHIP_REQ_EXTENSION   4

/**
 * Parameter address for module DYNAPSE_CONFIG_SYSINFO:
 * read-only parameter, the version of the logic currently
 * running on the device's FPGA/CPLD. It usually represents
 * a specific SVN revision, at which the logic code was
 * synthesized.
 * This is reserved for internal use and should not be used by
 * anything other than libcaer. Please see the 'struct caer_dynapse_info'
 * documentation to get this information.
 */
#define DYNAPSE_CONFIG_SYSINFO_LOGIC_VERSION    0
/**
 * Parameter address for module DYNAPSE_CONFIG_SYSINFO:
 * read-only parameter, an integer used to identify the different
 * types of sensor chips used on the device.
 * This is reserved for internal use and should not be used by
 * anything other than libcaer. Please see the 'struct caer_dynapse_info'
 * documentation to get this information.
 */
#define DYNAPSE_CONFIG_SYSINFO_CHIP_IDENTIFIER  1
/**
 * Parameter address for module DYNAPSE_CONFIG_SYSINFO:
 * read-only parameter, whether the device is currently a timestamp
 * master or slave when synchronizing multiple devices together.
 * This is reserved for internal use and should not be used by
 * anything other than libcaer. Please see the 'struct caer_dynapse_info'
 * documentation to get this information.
 */
#define DYNAPSE_CONFIG_SYSINFO_DEVICE_IS_MASTER 2
/**
 * Parameter address for module DYNAPSE_CONFIG_SYSINFO:
 * read-only parameter, the frequency in MHz at which the main
 * FPGA/CPLD logic is running.
 * This is reserved for internal use and should not be used by
 * anything other than libcaer. Please see the 'struct caer_dynapse_info'
 * documentation to get this information.
 */
#define DYNAPSE_CONFIG_SYSINFO_LOGIC_CLOCK      3

/**
 * Parameter address for module DYNAPSE_CONFIG_USB:
 * enable the USB FIFO module, which transfers the data from the
 * FPGA/CPLD to the USB chip, to be then sent to the host.
 * Turning this off will suppress any USB data communication!
 */
#define DYNAPSE_CONFIG_USB_RUN                0
/**
 * Parameter address for module DYNAPSE_CONFIG_USB:
 * the time delay after which a packet of data is committed to
 * USB, even if it is not full yet (short USB packet).
 * The value is in 125µs time-slices, corresponding to how
 * USB schedules its operations (a value of 4 for example
 * would mean waiting at most 0.5ms until sending a short
 * USB packet to the host).
 */
#define DYNAPSE_CONFIG_USB_EARLY_PACKET_DELAY 1

/**
 * Parameter address for module DYNAPSE_CONFIG_USB:
 * the time delay after which a packet of data is committed to
 * USB, even if it is not full yet (short USB packet).
 * The value is in 125µs time-slices, corresponding to how
 * USB schedules its operations (a value of 4 for example
 * would mean waiting at most 0.5ms until sending a short
 * USB packet to the host).
 */
#define DYNAPSE_CONFIG_SRAM_DIRECTION_POS 0
#define DYNAPSE_CONFIG_SRAM_DIRECTION_NEG 1

#define DYNAPSE_CONFIG_SRAM_DIRECTION_Y_NORTH 0
#define DYNAPSE_CONFIG_SRAM_DIRECTION_Y_SOUTH 1
#define DYNAPSE_CONFIG_SRAM_DIRECTION_X_EAST 0
#define DYNAPSE_CONFIG_SRAM_DIRECTION_X_WEST 1

/**
 * Parameter address for module DYNAPSE_X4BOARD_NEUX:
 * Number of neurons in the x direction of the board
 */
#define DYNAPSE_X4BOARD_NEUX 64
/**
 * Parameter address for module DYNAPSE_X4BOARD_NEUY:
 * Number of neurons in the x direction of the board
 */
#define DYNAPSE_X4BOARD_NEUY 64
/**
 * Parameter address for module DYNAPSE_X4BOARD_COREX:
 * Number of cores in the x direction of the board
 */
#define DYNAPSE_X4BOARD_COREX 4
/**
 * Parameter address for module DYNAPSE_X4BOARD_COREY:
 * Number of cores in the x direction of the board
 */
#define DYNAPSE_X4BOARD_COREY 4

#define DYNAPSE_CONFIG_DYNAPSE_U0	0
#define DYNAPSE_CONFIG_DYNAPSE_U1	8
#define DYNAPSE_CONFIG_DYNAPSE_U2	4
#define DYNAPSE_CONFIG_DYNAPSE_U3	12

#define DYNAPSE_CONFIG_NUMNEURONS			1024
#define DYNAPSE_CONFIG_SRAMROW				1024
#define DYNAPSE_CONFIG_CAMCOL				16
#define DYNAPSE_CONFIG_NUMNEURONS_CORE	 	256
#define DYNAPSE_CONFIG_NUMCORES				4
#define DYNAPSE_CONFIG_NUMSRAM_NEU	    	4
#define DYNAPSE_CONFIG_XCHIPSIZE   			32
#define DYNAPSE_CONFIG_YCHIPSIZE   			32
#define DYNAPSE_CONFIG_NEUROW				16
#define DYNAPSE_CONFIG_NEUCOL				16
#define DYNAPSE_CONFIG_NUMCAM				64

#define DYNAPSE_CONFIG_CAMTYPE_F_EXC		3
#define DYNAPSE_CONFIG_CAMTYPE_S_EXC		2
#define DYNAPSE_CONFIG_CAMTYPE_F_INH		1
#define DYNAPSE_CONFIG_CAMTYPE_S_INH		0

/*
 *  maximum user memory per query, libusb will digest it in chuncks of max 512 bytes per single transfer
 * */
#define DYNAPSE_MAX_USER_USB_PACKET_SIZE	1024
/*
 *  libusb max 512 bytes per single transfer
 * */
#define DYNAPSE_CONFIG_MAX_USB_TRANSFER 512
/*
 *  maximum number of 6 bytes configuration parameters, it needs to fit in 512
 * */
#define DYNAPSE_CONFIG_MAX_PARAM_SIZE 85

/**
 * Parameter address for module DYNAPSE_CONFIG_BIAS:
 * DYNAPSE chip biases.
 * Bias configuration values must be generated using the proper
 * functions, which are:
 * - convertBias() for coarse-fine (current) biases.
 * See 'http://inilabs.com/support/biasing/' for more details.
 */
#define DYNAPSE_CONFIG_BIAS_C0_PULSE_PWLK_P             	0
#define DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_INH_S_N            2
#define DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_INH_F_N            4
#define DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_EXC_S_N         	6
#define DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_EXC_F_N        	8
#define DYNAPSE_CONFIG_BIAS_C0_IF_RFR_N          			10
#define DYNAPSE_CONFIG_BIAS_C0_IF_TAU1_N         			12
#define DYNAPSE_CONFIG_BIAS_C0_IF_AHTAU_N           		14
#define DYNAPSE_CONFIG_BIAS_C0_IF_CASC_N 					16
#define DYNAPSE_CONFIG_BIAS_C0_IF_TAU2_N         			18
#define DYNAPSE_CONFIG_BIAS_C0_IF_BUF_P               		20
#define DYNAPSE_CONFIG_BIAS_C0_IF_AHTHR_N             		22
#define DYNAPSE_CONFIG_BIAS_C0_IF_THR_N            			24
#define DYNAPSE_CONFIG_BIAS_C0_NPDPIE_THR_S_P             	26
#define DYNAPSE_CONFIG_BIAS_C0_NPDPIE_THR_F_P            	28
#define DYNAPSE_CONFIG_BIAS_C0_NPDPII_THR_F_P      			30
#define DYNAPSE_CONFIG_BIAS_C0_NPDPII_THR_S_P            	32
#define DYNAPSE_CONFIG_BIAS_C0_IF_NMDA_N            		34
#define DYNAPSE_CONFIG_BIAS_C0_IF_DC_P           			36
#define DYNAPSE_CONFIG_BIAS_C0_IF_AHW_P          			38
#define DYNAPSE_CONFIG_BIAS_C0_NPDPII_TAU_S_P          		40
#define DYNAPSE_CONFIG_BIAS_C0_NPDPII_TAU_F_P 				42
#define DYNAPSE_CONFIG_BIAS_C0_NPDPIE_TAU_F_P         		44
#define DYNAPSE_CONFIG_BIAS_C0_NPDPIE_TAU_S_P               46
#define DYNAPSE_CONFIG_BIAS_C0_R2R_P               			48

#define DYNAPSE_CONFIG_BIAS_C1_PULSE_PWLK_P             	1
#define DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_INH_S_N            3
#define DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_INH_F_N            5
#define DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_EXC_S_N         	7
#define DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_EXC_F_N        	9
#define DYNAPSE_CONFIG_BIAS_C1_IF_RFR_N          			11
#define DYNAPSE_CONFIG_BIAS_C1_IF_TAU1_N         			13
#define DYNAPSE_CONFIG_BIAS_C1_IF_AHTAU_N           		15
#define DYNAPSE_CONFIG_BIAS_C1_IF_CASC_N 					17
#define DYNAPSE_CONFIG_BIAS_C1_IF_TAU2_N         			19
#define DYNAPSE_CONFIG_BIAS_C1_IF_BUF_P               		21
#define DYNAPSE_CONFIG_BIAS_C1_IF_AHTHR_N             		23
#define DYNAPSE_CONFIG_BIAS_C1_IF_THR_N            			25
#define DYNAPSE_CONFIG_BIAS_C1_NPDPIE_THR_S_P             	27
#define DYNAPSE_CONFIG_BIAS_C1_NPDPIE_THR_F_P            	29
#define DYNAPSE_CONFIG_BIAS_C1_NPDPII_THR_F_P      			31
#define DYNAPSE_CONFIG_BIAS_C1_NPDPII_THR_S_P            	33
#define DYNAPSE_CONFIG_BIAS_C1_IF_NMDA_N            		35
#define DYNAPSE_CONFIG_BIAS_C1_IF_DC_P           			37
#define DYNAPSE_CONFIG_BIAS_C1_IF_AHW_P          			39
#define DYNAPSE_CONFIG_BIAS_C1_NPDPII_TAU_S_P          		41
#define DYNAPSE_CONFIG_BIAS_C1_NPDPII_TAU_F_P 				43
#define DYNAPSE_CONFIG_BIAS_C1_NPDPIE_TAU_F_P         		45
#define DYNAPSE_CONFIG_BIAS_C1_NPDPIE_TAU_S_P               47
#define DYNAPSE_CONFIG_BIAS_C1_R2R_P               			49

#define DYNAPSE_CONFIG_BIAS_U_BUFFER         				50
#define DYNAPSE_CONFIG_BIAS_U_SSP               			51
#define DYNAPSE_CONFIG_BIAS_U_SSN               			52

#define DYNAPSE_CONFIG_BIAS_C2_PULSE_PWLK_P             	64
#define DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_INH_S_N            66
#define DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_INH_F_N            68
#define DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_EXC_S_N         	70
#define DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_EXC_F_N        	72
#define DYNAPSE_CONFIG_BIAS_C2_IF_RFR_N          			74
#define DYNAPSE_CONFIG_BIAS_C2_IF_TAU1_N         			76
#define DYNAPSE_CONFIG_BIAS_C2_IF_AHTAU_N           		78
#define DYNAPSE_CONFIG_BIAS_C2_IF_CASC_N 					80
#define DYNAPSE_CONFIG_BIAS_C2_IF_TAU2_N         			82
#define DYNAPSE_CONFIG_BIAS_C2_IF_BUF_P               		84
#define DYNAPSE_CONFIG_BIAS_C2_IF_AHTHR_N             		86
#define DYNAPSE_CONFIG_BIAS_C2_IF_THR_N            			88
#define DYNAPSE_CONFIG_BIAS_C2_NPDPIE_THR_S_P             	90
#define DYNAPSE_CONFIG_BIAS_C2_NPDPIE_THR_F_P            	92
#define DYNAPSE_CONFIG_BIAS_C2_NPDPII_THR_F_P      			94
#define DYNAPSE_CONFIG_BIAS_C2_NPDPII_THR_S_P            	96
#define DYNAPSE_CONFIG_BIAS_C2_IF_NMDA_N            		98
#define DYNAPSE_CONFIG_BIAS_C2_IF_DC_P           			100
#define DYNAPSE_CONFIG_BIAS_C2_IF_AHW_P          			102
#define DYNAPSE_CONFIG_BIAS_C2_NPDPII_TAU_S_P          		104
#define DYNAPSE_CONFIG_BIAS_C2_NPDPII_TAU_F_P 				106
#define DYNAPSE_CONFIG_BIAS_C2_NPDPIE_TAU_F_P         		108
#define DYNAPSE_CONFIG_BIAS_C2_NPDPIE_TAU_S_P               110
#define DYNAPSE_CONFIG_BIAS_C2_R2R_P               			112

#define DYNAPSE_CONFIG_BIAS_C3_PULSE_PWLK_P             	65
#define DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_INH_S_N            67
#define DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_INH_F_N            69
#define DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_EXC_S_N         	71
#define DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_EXC_F_N        	73
#define DYNAPSE_CONFIG_BIAS_C3_IF_RFR_N          			75
#define DYNAPSE_CONFIG_BIAS_C3_IF_TAU1_N         			77
#define DYNAPSE_CONFIG_BIAS_C3_IF_AHTAU_N           		79
#define DYNAPSE_CONFIG_BIAS_C3_IF_CASC_N 					81
#define DYNAPSE_CONFIG_BIAS_C3_IF_TAU2_N         			83
#define DYNAPSE_CONFIG_BIAS_C3_IF_BUF_P               		85
#define DYNAPSE_CONFIG_BIAS_C3_IF_AHTHR_N             		87
#define DYNAPSE_CONFIG_BIAS_C3_IF_THR_N            			89
#define DYNAPSE_CONFIG_BIAS_C3_NPDPIE_THR_S_P             	91
#define DYNAPSE_CONFIG_BIAS_C3_NPDPIE_THR_F_P            	93
#define DYNAPSE_CONFIG_BIAS_C3_NPDPII_THR_F_P      			95
#define DYNAPSE_CONFIG_BIAS_C3_NPDPII_THR_S_P            	97
#define DYNAPSE_CONFIG_BIAS_C3_IF_NMDA_N            		99
#define DYNAPSE_CONFIG_BIAS_C3_IF_DC_P           			101
#define DYNAPSE_CONFIG_BIAS_C3_IF_AHW_P          			103
#define DYNAPSE_CONFIG_BIAS_C3_NPDPII_TAU_S_P          		105
#define DYNAPSE_CONFIG_BIAS_C3_NPDPII_TAU_F_P 				107
#define DYNAPSE_CONFIG_BIAS_C3_NPDPIE_TAU_F_P         		109
#define DYNAPSE_CONFIG_BIAS_C3_NPDPIE_TAU_S_P               111
#define DYNAPSE_CONFIG_BIAS_C3_R2R_P               			113

#define DYNAPSE_CONFIG_BIAS_D_BUFFER         				114
#define DYNAPSE_CONFIG_BIAS_D_SSP               			115
#define DYNAPSE_CONFIG_BIAS_D_SSN               			116

/**
 * Dynap-se device-related information.
 */
struct caer_dynapse_info {
	/// Unique device identifier. Also 'source' for events.
	int16_t deviceID;
	/// Device serial number.
	char deviceSerialNumber[8 + 1];
	/// Device USB bus number.
	uint8_t deviceUSBBusNumber;
	/// Device USB device address.
	uint8_t deviceUSBDeviceAddress;
	/// Device information string, for logging purposes.
	char *deviceString;
	/// Logic (FPGA/CPLD) version.
	int16_t logicVersion;
	/// Whether the device is a time-stamp master or slave.
	bool deviceIsMaster;
	/// Clock in MHz for main logic (FPGA/CPLD).
	int16_t logicClock;
	/// Chip identifier/type.
	int16_t chipID;
};

/**
 * On-chip coarse-fine bias current configuration.
 * See 'http://inilabs.com/support/biasing/' for more details.
 */
struct caer_bias_dynapse {
	/// Coarse current, from 0 to 7, creates big variations in output current.
	uint8_t coarseValue;
	/// Fine current, from 0 to 255, creates small variations in output current.
	uint8_t fineValue;
	/// Whether this bias is enabled or not.
	bool enabled;
	/// Bias sex: true for 'N' type, false for 'P' type.
	bool sexN;
	/// Bias current level: true for 'Normal, false for 'Low'.
	bool currentLevelNormal;
	/// Bias current level: true for 'HighBias', false for 'LowBias'.
	bool BiasLowHi;
	/// whether this is a special bias.
	bool special;
};
// TODO: what? precise biasLowHi, currentLevel really, special?
// TODO: add generate/parse functions.

/**
 * Return basic information on the device, such as its ID, the logic
 * version, and so on. See the 'struct caer_dynapse_info' documentation
 * for more details.
 *
 * @param handle a valid device handle.
 *
 * @return a copy of the device information structure if successful,
 *         an empty structure (all zeros) on failure.
 */
struct caer_dynapse_info caerDynapseInfoGet(caerDeviceHandle handle);

/*
 * @param cdh a valid device handle
 * Transfer numWords 16 bit words from *data to SRAM start at
 * address baseAddr in SRAM.
 * @return true on success, false otherwise
 */
bool caerDynapseWriteSramWords(caerDeviceHandle handle, const uint16_t *data, uint32_t baseAddr, uint32_t numWords);

/*
 *
 *  Remember to Select the chip before calling this function
 * @param handle a valid device handle.
 *
 *  coreId [0,3]
 *  neuronId [0,255], virtualCoreId [0,3],
 *  sx [DYNAPSE_CONFIG_SRAM_DIRECTION_X_EAST,DYNAPSE_CONFIG_SRAM_DIRECTION_X_WEST], dx,
 *  sy[DYNAPSE_CONFIG_SRAM_DIRECTION_Y_NORTH,DYNAPSE_CONFIG_SRAM_DIRECTION_Y_SOUTH], dy,
 *  sramId [0,3],
 *  destinationCore [0,0,0,0]...[1,1,1,1] hot coded 15 (all cores)
 *
 * @return true on success, false otherwise
 */
bool caerDynapseWriteSram(caerDeviceHandle handle, uint16_t coreId, uint32_t neuronId, uint16_t virtualCoreId, bool sx,
	uint8_t dx, bool sy, uint8_t dy, uint16_t sramId, uint16_t destinationCore);

/*
 * Remember to Select the chip before calling this function
 *
 * @param handle a valid device handle.
 *  int *data , pointer to array of integers bits
 *  numConfig , number of configurations to sends
 *
 *  Copy send data[numConfig] via usb
 *  NB: Make sure that data has max size data[DYNAPSE_SPIKE_DEFAULT_SIZE]
 *
 * @return true on success, false otherwise
 */
bool caerDynapseSendDataToUSB(caerDeviceHandle handle, const uint32_t *data, size_t numConfig);

/*
 * Remember to Select the chip before calling this function
 *
 * @param handle a valid device handle.
 *  Write a single CAM
 *
 *  parameters:
 *	usb_handle, preNeuron [0,1023], postNeuron [0,1023], camId [0,63], synapseType [DYNAPSE_CONFIG_CAMTYPE_F_EXC
 *																					DYNAPSE_CONFIG_CAMTYPE_S_EXC
 *																					DYNAPSE_CONFIG_CAMTYPE_F_INH
 *																					DYNAPSE_CONFIG_CAMTYPE_S_INH]
 * @return true on success, false otherwise
 */
bool caerDynapseWriteCam(caerDeviceHandle handle, uint32_t preNeuronAddr, uint32_t postNeuronAddr, uint32_t camId,
	int16_t synapseType);

/*
 * @param handle a valid device handle.
 *  Return addres for writing CAM
 *
 *  parameters:
 *	usb_handle, preNeuron [0,1023], postNeuron [0,1023], camId [0,63], synapseType [DYNAPSE_CONFIG_CAMTYPE_F_EXC
 *																					DYNAPSE_CONFIG_CAMTYPE_S_EXC
 *																					DYNAPSE_CONFIG_CAMTYPE_F_INH
 *																					DYNAPSE_CONFIG_CAMTYPE_S_INH]
 *
 * @return bits that would make the connection
 */
uint32_t caerDynapseGenerateCamBits(uint32_t preNeuronAddr, uint32_t postNeuronAddr, uint32_t camId,
	int16_t synapseType);

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_DEVICES_DYNAPSE_H_ */
