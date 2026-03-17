// #ifndef _QUADDECODE_H_
// #define _QUADDECODE_H_

// #include <Arduino.h>
// #include <stdint.h>
// /*
//  *  "Ecrit" par Etienne ARLAUD

//  *  Le code a salement été pompé sur ces 2 sites :
//  *  Pour le fonctionnement du FTM :
//  *  https://forum.pjrc.com/threads/26803-Hardware-Quadrature-Code-for-Teensy-3-x/page3
//  *
//  *  Pour la mise en place d'interruptions :
//  *  https://forum.pjrc.com/threads/40173-FTM-Channel-compare-interrupts-(Teensy-3-5)
//  *
//  *
//  *  Finalement, le plus gros a été pompé d'ici :
//  *  https://forum.pjrc.com/threads/40825-T3-6-Using-an-external-clock-for-FTM-timers
//  *
//  *  Pour rappel la datasheet du proco de teensy 3.2 :
//  *  https://www.pjrc.com/teensy/K20P64M72SF1RM.pdf
//  *
//  */

// // classe permettant de dechiffrer les odometres

// class QuadDecode final {
//   public:
//     QuadDecode(const QuadDecode &) = delete;
//     QuadDecode(QuadDecode &) = delete;
//     QuadDecode(const QuadDecode &&) = delete;
//     QuadDecode(QuadDecode &&) = delete;

//     QuadDecode &operator=(const QuadDecode &) = delete;
//     QuadDecode &operator=(QuadDecode &&) = delete;

//     int16_t retenue1;
//     int16_t retenue2;

//     bool first_interrupt_1;
//     bool first_interrupt_2;

//     void resetCounters() {
//         resetCounter1();
//         resetCounter2();
//     }
//     int32_t getLeftCounter() const { return -getCounter2(); }
//     int32_t getRightCounter() const { return getCounter1(); }

//     static QuadDecode &instance() {
//         static QuadDecode instance;
//         return instance;
//     }

//   private:
//     QuadDecode() {
//         retenue1 = 0;
//         retenue2 = 0;

//         first_interrupt_1 = true;
//         first_interrupt_2 = true;

//         // initialise FTM_CLKIN0 and Flextimer 1 CH0
//         FTM1_MODE = 0x05;  // set write-protect disable (WPDIS) bit to modify other registers
//                            // FAULTIE=0, FAULTM=00, CAPTEST=0, PWMSYNC=0, WPDIS=1, INIT=0, FTMEN=1(no restriction FTM)
//         FTM1_SC = 0x00;    // set FTM1 Status/Control to zero = disabled (enabled later in setup)
//         FTM1_CNT = 0x0000; // reset count to zero
//         FTM1_MOD = 0xFFFF; // Maximum value of the counter
//         FTM1_C0SC = 0x14;  // CHF=0, CHIE=0 (disable interrupt, use software polling), MSB=0 MSA=1, ELSB=0 ELSA=1 (output compare - toggle), 0, DMA=0

//         FTM2_MODE = 0x05;  // set write-protect disable (WPDIS) bit to modify other registers
//                            // FAULTIE=0, FAULTM=00, CAPTEST=0, PWMSYNC=0, WPDIS=1, INIT=0, FTMEN=1(no restriction FTM)
//         FTM2_SC = 0x00;    // set FTM1 Status/Control to zero = disabled (enabled later in setup)
//         FTM2_CNT = 0x0000; // reset count to zero
//         FTM2_MOD = 0xFFFF; // Maximum value of the counter
//         FTM2_C0SC = 0x14;  // CHF=0, CHIE=0 (disable interrupt, use software polling), MSB=0 MSA=1, ELSB=0 ELSA=1 (output compare - toggle), 0, DMA=0

//         // Set registers to count quadrature
//         // FTM1_FILTER=0x22;	// 2x4 clock filters on both channels
//         FTM1_CNTIN = 0x0000;
//         FTM1_QDCTRL = 0b11000001;

//         // FTM2_FILTER=0x22;	// 2x4 clock filters on both channels
//         FTM2_CNTIN = 0x0000;
//         FTM2_QDCTRL = 0b11000001;

//         // enable FTM interrupt within NVIC table
//         NVIC_ENABLE_IRQ(IRQ_FTM1);
//         NVIC_ENABLE_IRQ(IRQ_FTM2);

//         // configure Teensy port pins
//         PORTA_PCR12 = 0x00000712; // Alt7-QD_FTM1,FilterEnable,Pulldown
//         PORTA_PCR13 = 0x00000712;
//         // configure Teensy port pins
//         PORTB_PCR18 = 0x00000612; // Alt7-QD_FTM1,FilterEnable,Pulldown
//         PORTB_PCR19 = 0x00000612;

//         // enable external clock (10 MHz), no prescale
//         FTM1_C0V = 0;   // compare value = 0
//         FTM1_SC = 0x58; //(Note - FTM1_SC [TOF=0 TOIE=1 CPWMS=0 CLKS=11 (external clocks enabled) PS=000 [no prescale divide])

//         // enable external clock (10 MHz), no prescale
//         FTM2_C0V = 0;   // compare value = 0
//         FTM2_SC = 0x58; //(Note - FTM1_SC [TOF=0 TOIE=1 CPWMS=0 CLKS=11 (external clocks enabled) PS=000 [no prescale divide])
//     }

//     void resetCounter1() {
//         FTM1_CNT = 0xFFFF; // When we try to write CNT, CNT take the value of CNTIN (aka 0x0000 here) !
//         retenue1 = 0;
//     }

//     void resetCounter2() {
//         FTM2_CNT = 0xFFFF; // When we try to write CNT, CNT take the value of CNTIN (aka 0x0000 here) !
//         retenue2 = 0;
//     }

//     int32_t getCounter1() const {
//         int32_t c = FTM1_CNT + 0x10000 * retenue1;
//         return c;
//     }

//     int32_t getCounter2() const {
//         int32_t c = (uint32_t)FTM2_CNT + (uint32_t)0x10000 * retenue2;

//         return c;
//     }
// };

// /**
//  * Wrapper around the singleton instance of QuadDecode. This allows type `encoders_t` to be default-constructible.
//  *
//  * All the instances of this class delegate to the same QuadDecode object. This class is not thread-safe.
//  */
// class QuadDecodeRef {
//   public:
//     QuadDecodeRef() : m_reference(QuadDecode::instance()) {}

//     void resetCounters() { m_reference.resetCounters(); }
//     int32_t getLeftCounter() const { return m_reference.getLeftCounter(); }
//     int32_t getRightCounter() const { return m_reference.getRightCounter(); }

//   private:
//     /// Static reference
//     QuadDecode &m_reference;
// };

// extern "C" void ftm1_isr(void);
// extern "C" void ftm2_isr(void);

// #endif


#ifndef _QUADDECODE_H_
#define _QUADDECODE_H_

#include <Arduino.h>
#include "QuadEncoder.h" // The header from the library you found

class QuadDecode final {
  public:
    QuadDecode(const QuadDecode &) = delete;
    QuadDecode &operator=(const QuadDecode &) = delete;

    // Dummy variables to keep compatibility with your old code
    int16_t retenue1 = 0;
    int16_t retenue2 = 0;
    bool first_interrupt_1 = false;
    bool first_interrupt_2 = false;

    static QuadDecode &instance() {
        static QuadDecode instance;
        return instance;
    }

    void resetCounters() {
        enc1.write(0);
        enc2.write(0);
    }

    // Adjust the minus signs if your robot moves backwards
    int32_t getLeftCounter() const { return -enc2.read(); }
    int32_t getRightCounter() const { return enc1.read(); }

  private:
    /* Teensy 4.0 Hardware Encoder Mapping:
       Module 1 uses Phase A on Pin 0 and Phase B on Pin 1
       Module 2 uses Phase A on Pin 3 and Phase B on Pin 4 
    */
    mutable QuadEncoder enc1; // Hardware Encoder Module 1
    mutable QuadEncoder enc2; // Hardware Encoder Module 2

    QuadDecode() : enc1(1, 0, 1), enc2(2, 3, 4) {
        // Constructor arguments: (Module Number, Pin A, Pin B)
        enc1.setInitConfig();
        enc1.init();
        
        enc2.setInitConfig();
        enc2.init();
    }
};

class QuadDecodeRef {
  public:
    QuadDecodeRef() : m_reference(QuadDecode::instance()) {}
    void resetCounters() { m_reference.resetCounters(); }
    int32_t getLeftCounter() const { return m_reference.getLeftCounter(); }
    int32_t getRightCounter() const { return m_reference.getRightCounter(); }
  private:
    QuadDecode &m_reference;
};

#endif