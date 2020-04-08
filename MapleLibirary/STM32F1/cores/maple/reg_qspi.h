//-----------------------------------------------------------------------------
// The confidential and proprietary information contained in this file may     
// only be used by a person authorised under and to the extent permitted       
// by a subsisting licensing agreement from FASTASIC Limited.              
//                                                                             
//            (C) COPYRIGHT 2015-2017 FASTASIC Limited.                  
//                ALL RIGHTS RESERVED                                          
//                                                                             
// This entire notice must be reproduced on all copies of this file            
// and copies of this file may only be made by a person if such person is      
// permitted to do so under the terms of a subsisting license agreement        
// from FASTASIC Limited.                                                  
//-----------------------------------------------------------------------------
#ifndef __REG_QSPI_H__
#define __REG_QSPI_H__
//Auto-gen by fr
#include "_reg_base_addr.h"
//ctrlr0
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                     reserved_1 :  4; /* 3: 0,                           rsvd*/
        uint32_t                            frf :  2; /* 5: 4,  Frame Format. Selects which serial protocol transfers the data.
        uint32_t                           scph :  1; /* 6: 6, Serial Clock Phase. Valid when the frame format (FRF) is set to Motorola SPI. The
        uint32_t                          scpol :  1; /* 7: 7, Serial Clock Polarity. Valid when the frame format (FRF) is set to Motorola SPI.
        uint32_t                           tmod :  2; /* 9: 8, Transfer Mode. Selects the mode of transfer for serial communication. This field
        uint32_t                         slv_oe :  1; /*10:10, Slave Output Enable. Relevant only when the DW_apb_ssi is configured as a
        uint32_t                            srl :  1; /*11:11, Shift Register Loop. Used for testing purposes only. When internally active,
        uint32_t                            cfs :  4; /*15:12, Control Frame Size. Selects the length of the control word for the Microwire frame
        uint32_t                            dfs :  5; /*20:16, Data Frame Size in 32-bit mode
        uint32_t                        spi_frf :  2; /*22:21, SPI Frame Format:
        uint32_t                     reserved_0 :  9; /*31:23,                           rsvd*/
    } bit_field;
} T_QSPI_CTRLR0;

//ctrlr1
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                            ndf : 16; /*15: 0, Number of Data Frames. When TMOD = 10 or TMOD = 11, this register field sets the
        uint32_t                     reserved_0 : 16; /*31:16,                             NA*/
    } bit_field;
} T_QSPI_CTRLR1;

//ssienr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                         ssi_en :  1; /* 0: 0, SSI Enable. Enables and disables all DW_apb_ssi operations. When disabled, all serial
        uint32_t                     reserved_0 : 31; /*31: 1,                             NA*/
    } bit_field;
} T_QSPI_SSIENR;

//mwcr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                          mwmod :  1; /* 0: 0, Microwire Transfer Mode. Defines whether the Microwire transfer is sequential or
        uint32_t                            mod :  1; /* 1: 1, Microwire Control. Defines the direction of the data word when the Microwire serial
        uint32_t                            mhs :  1; /* 2: 2, Microwire Handshaking. Relevant only when the DW_apb_ssi is configured as a
        uint32_t                     reserved_0 : 29; /*31: 3,                             NA*/
    } bit_field;
} T_QSPI_MWCR;

//ser
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                            ser :  1; /* 0: 0, Slave Select Enable Flag. Each bit in this register corresponds to a
        uint32_t                     reserved_0 : 31; /*31: 1,                             NA*/
    } bit_field;
} T_QSPI_SER;

//baudr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                          sckdv : 16; /*15: 0, SSI Clock Divider. The LSB for this field is always set to 0 and is unaffected by a write
        uint32_t                     reserved_0 : 16; /*31:16,                             NA*/
    } bit_field;
} T_QSPI_BAUDR;

//txftlr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                            tft :  6; /* 5: 0, Transmit FIFO Threshold. Controls the level of entries (or below) at which the
        uint32_t                     reserved_0 : 26; /*31: 6,                             NA*/
    } bit_field;
} T_QSPI_TXFTLR;

//rxftlr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                            rft :  6; /* 5: 0, Receive FIFO Threshold. Controls the level of entries (or above) at
        uint32_t                     reserved_0 : 26; /*31: 6,                             NA*/
    } bit_field;
} T_QSPI_RXFTLR;

//txflr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                          txtfl :  7; /* 6: 0, Transmit FIFO Level. Contains the number of valid data entries in the
        uint32_t                     reserved_0 : 25; /*31: 7,                             NA*/
    } bit_field;
} T_QSPI_TXFLR;

//rxflr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                          rxtfl :  7; /* 6: 0, Receive FIFO Level. Contains the number of valid data entries in the
        uint32_t                     reserved_0 : 25; /*31: 7,                             NA*/
    } bit_field;
} T_QSPI_RXFLR;

//sr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                           busy :  1; /* 0: 0, SSI Busy Flag. When set, indicates that a serial transfer is in progress; when cleared
        uint32_t                           tfnf :  1; /* 1: 1, Transmit FIFO Not Full. Set when the transmit FIFO contains one or more empty locations,
        uint32_t                            tfe :  1; /* 2: 2, Transmit FIFO Empty. When the transmit FIFO is completely empty, this bit is set. When
        uint32_t                           rfne :  1; /* 3: 3, Receive FIFO Not Empty. Set when the receive FIFO contains one or more entries and is
        uint32_t                            rff :  1; /* 4: 4, Receive FIFO Full. When the receive FIFO is completely full, this bit is set. When the
        uint32_t                            txe :  1; /* 5: 5, Transmission Error. Set if the transmit FIFO is empty when a transfer is started. This bit can
        uint32_t                           dcol :  1; /* 6: 6, Data Collision Error. Relevant only when the DW_apb_ssi is configured as a master device.
        uint32_t                     reserved_0 : 25; /*31: 7,                             NA*/
    } bit_field;
} T_QSPI_SR;

//imr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                          txeim :  1; /* 0: 0, Transmit FIFO Empty Interrupt Mask
        uint32_t                          txoim :  1; /* 1: 1, Transmit FIFO Overflow Interrupt Mask
        uint32_t                          rxuim :  1; /* 2: 2, Receive FIFO Underflow Interrupt Mask
        uint32_t                          rxoim :  1; /* 3: 3, Receive FIFO Overflow Interrupt Mask
        uint32_t                          rxfim :  1; /* 4: 4, Receive FIFO Full Interrupt Mask
        uint32_t                          mstim :  1; /* 5: 5, Multi-Master Contention Interrupt Mask. This bit field is not present if the DW_apb_ssi
        uint32_t                     reserved_0 : 26; /*31: 6,                             NA*/
    } bit_field;
} T_QSPI_IMR;

//isr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                          txeis :  1; /* 0: 0, Transmit FIFO Empty Interrupt Status
        uint32_t                          txois :  1; /* 1: 1, Transmit FIFO Overflow Interrupt Status
        uint32_t                          rxuis :  1; /* 2: 2, Receive FIFO Underflow Interrupt Status
        uint32_t                          rxois :  1; /* 3: 3, Receive FIFO Overflow Interrupt Status
        uint32_t                          rxfis :  1; /* 4: 4, Receive FIFO Full Interrupt Status
        uint32_t                          mstis :  1; /* 5: 5, Multi-Master Contention Interrupt Status. This bit field is not present if the
        uint32_t                     reserved_0 : 26; /*31: 6,                             NA*/
    } bit_field;
} T_QSPI_ISR;

//risr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                          txeir :  1; /* 0: 0, Transmit FIFO Empty Raw Interrupt Status
        uint32_t                          txoir :  1; /* 1: 1, Transmit FIFO Overflow Raw Interrupt Status
        uint32_t                          rxuir :  1; /* 2: 2, Receive FIFO Underflow Raw Interrupt Status
        uint32_t                          rxoir :  1; /* 3: 3, Receive FIFO Overflow Raw Interrupt Status
        uint32_t                          rxfir :  1; /* 4: 4, Receive FIFO Full Raw Interrupt Status
        uint32_t                          mstir :  1; /* 5: 5, Multi-Master Contention Raw Interrupt Status. This bit field is not present if the
        uint32_t                     reserved_0 : 26; /*31: 6,                             NA*/
    } bit_field;
} T_QSPI_RISR;

//txoicr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                         txoicr :  1; /* 0: 0, Clear Transmit FIFO Overflow Interrupt. This register reflects the status of the
        uint32_t                     reserved_0 : 31; /*31: 1,                             NA*/
    } bit_field;
} T_QSPI_TXOICR;

//rxoicr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                         rxoicr :  1; /* 0: 0, Clear Receive FIFO Overflow Interrupt. This register reflects the status of the interrupt.
        uint32_t                     reserved_0 : 31; /*31: 1,                             NA*/
    } bit_field;
} T_QSPI_RXOICR;

//rxuicr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                         rxuicr :  1; /* 0: 0, Clear Receive FIFO Underflow Interrupt. This register reflects the status of the interrupt.
        uint32_t                     reserved_0 : 31; /*31: 1,                             NA*/
    } bit_field;
} T_QSPI_RXUICR;

//msticr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                         msticr :  1; /* 0: 0, Clear Multi-Master Contention Interrupt. This register reflects the status of the interrupt.
        uint32_t                     reserved_0 : 31; /*31: 1,                             NA*/
    } bit_field;
} T_QSPI_MSTICR;

//icr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                            icr :  1; /* 0: 0, Clear Interrupts. This register is set if any of the interrupts below are active. A read
        uint32_t                     reserved_0 : 31; /*31: 1,                             NA*/
    } bit_field;
} T_QSPI_ICR;

//dmacr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                          rdmae :  1; /* 0: 0, Receive DMA Enable. This bit enables/disables the receive FIFO DMA channel
        uint32_t                          tdmae :  1; /* 1: 1, Transmit DMA Enable. This bit enables/disables the transmit FIFO DMA channel.
        uint32_t                     reserved_0 : 30; /*31: 2,                             NA*/
    } bit_field;
} T_QSPI_DMACR;

//dmatdlr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                         dmatdl :  6; /* 5: 0, Transmit Data Level. This bit field controls the level at which a DMA
        uint32_t                     reserved_0 : 26; /*31: 6,                             NA*/
    } bit_field;
} T_QSPI_DMATDLR;

//dmardlr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                         dmardl :  6; /* 5: 0, Receive Data Level. This bit field controls the level at which a DMA request
        uint32_t                     reserved_0 : 26; /*31: 6,                             NA*/
    } bit_field;
} T_QSPI_DMARDLR;

//idr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                         idcode : 32; /*31: 0, Identification Code. This register contains the peripherals identification code, which is
    } bit_field;
} T_QSPI_IDR;

//ssi_comp_version
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t               ssi_comp_version : 32; /*31: 0, Contains the hex representation of the Synopsys component version.
    } bit_field;
} T_QSPI_SSI_COMP_VERSION;

//dr
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                             dr : 32; /*31: 0, Data Register. When writing to this register, you must right-justify the data. Read
    } bit_field;
} T_QSPI_DR;

//dr_reversed
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                    dr_reversed : 32; /*31: 0, Data Register. When writing to this register, you must right-justify the data. Read
    } bit_field;
} T_QSPI_DR_REVERSED;

//rxsample_dly
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                            rsd :  8; /* 7: 0,                             NA*/
        uint32_t                     reserved_0 : 24; /*31: 8,                             NA*/
    } bit_field;
} T_QSPI_RXSAMPLE_DLY;

//spi_ctrlr0
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t                     trans_type :  2; /* 1: 0, Address and instruction transfer format.
        uint32_t                         addr_l :  4; /* 5: 2, This bit defines length of address to be transmitted. See Table 6-10 for the
        uint32_t                     reserved_2 :  2; /* 7: 6,                             NA*/
        uint32_t                         inst_l :  2; /* 9: 8, Dual/Quad mode instruction length in bits.
        uint32_t                     reserved_1 :  1; /*10:10,                             NA*/
        uint32_t                    wait_cycles :  4; /*14:11, This bit defines the wait cycles in dual/quad mode between control frames
        uint32_t                     reserved_0 : 17; /*31:15,                             NA*/
    } bit_field;
} T_QSPI_SPI_CTRLR0;

//Registers Mapping to the same address

typedef struct
{
    volatile                   T_QSPI_CTRLR0                         ctrlr0; /*  0x0,    RW, 0x00070000,                             NA*/
    volatile                   T_QSPI_CTRLR1                         ctrlr1; /*  0x4,    RW, 0x00000000,                             NA*/
    volatile                   T_QSPI_SSIENR                         ssienr; /*  0x8,    RW, 0x00000000,                             NA*/
    volatile                     T_QSPI_MWCR                           mwcr; /*  0xc,    RW, 0x00000000,                             NA*/
    volatile                      T_QSPI_SER                            ser; /* 0x10,    RW, 0x00000000,                             NA*/
    volatile                    T_QSPI_BAUDR                          baudr; /* 0x14,    RW, 0x00000000,                             NA*/
    volatile                   T_QSPI_TXFTLR                         txftlr; /* 0x18,    RW, 0x00000000,                             NA*/
    volatile                   T_QSPI_RXFTLR                         rxftlr; /* 0x1c,    RW, 0x00000000,                             NA*/
    volatile                    T_QSPI_TXFLR                          txflr; /* 0x20,    RO, 0x00000000,                             NA*/
    volatile                    T_QSPI_RXFLR                          rxflr; /* 0x24,    RO, 0x00000000,                             NA*/
    volatile                       T_QSPI_SR                             sr; /* 0x28,    RO, 0x00000006,                             NA*/
    volatile                      T_QSPI_IMR                            imr; /* 0x2c,    RW, 0x0000003F,                             NA*/
    volatile                      T_QSPI_ISR                            isr; /* 0x30,    RO, 0x00000000,                             NA*/
    volatile                     T_QSPI_RISR                           risr; /* 0x34,    RO, 0x00000000,                             NA*/
    volatile                   T_QSPI_TXOICR                         txoicr; /* 0x38,    RO, 0x00000000,                             NA*/
    volatile                   T_QSPI_RXOICR                         rxoicr; /* 0x3c,    RO, 0x00000000,                             NA*/
    volatile                   T_QSPI_RXUICR                         rxuicr; /* 0x40,    RO, 0x00000000,                             NA*/
    volatile                   T_QSPI_MSTICR                         msticr; /* 0x44,    RO, 0x00000000,                             NA*/
    volatile                      T_QSPI_ICR                            icr; /* 0x48,    RO, 0x00000000,                             NA*/
    volatile                    T_QSPI_DMACR                          dmacr; /* 0x4c,    RW, 0x00000000,                             NA*/
    volatile                  T_QSPI_DMATDLR                        dmatdlr; /* 0x50,    RW, 0x00000000,                             NA*/
    volatile                  T_QSPI_DMARDLR                        dmardlr; /* 0x54,    RW, 0x00000000,                             NA*/
    volatile                      T_QSPI_IDR                            idr; /* 0x58,    RO, 0xFFFFFFFF,                             NA*/
    volatile         T_QSPI_SSI_COMP_VERSION               ssi_comp_version; /* 0x5c,    RO, 0x3430302A,                             NA*/
    volatile                       T_QSPI_DR                             dr; /* 0x60,    RW, 0x00000000,                             NA*/
    volatile                        uint32_t                 reserved_0[34];
    volatile              T_QSPI_DR_REVERSED                    dr_reversed; /* 0xec,    RW, 0x00000000,                             NA*/
    volatile             T_QSPI_RXSAMPLE_DLY                   rxsample_dly; /* 0xf0,    RW, 0x00000000,                             NA*/
    volatile               T_QSPI_SPI_CTRLR0                     spi_ctrlr0; /* 0xf4,    RW, 0x00000200,                             NA*/
} T_HWP_QSPI_T;

#define hwp_qspi ((T_HWP_QSPI_T*)REG_QSPI_BASE)


__STATIC_INLINE uint32_t qspi_ctrlr0_get(void)
{
    return hwp_qspi->ctrlr0.val;
}

__STATIC_INLINE void qspi_ctrlr0_set(uint32_t value)
{
    hwp_qspi->ctrlr0.val = value;
}

__STATIC_INLINE void qspi_ctrlr0_pack(uint8_t spi_frf, uint8_t dfs, uint8_t cfs, uint8_t srl, uint8_t slv_oe, uint8_t tmod, uint8_t scpol, uint8_t scph, uint8_t frf)
{
    hwp_qspi->ctrlr0.val = (((uint32_t)spi_frf << 21) | ((uint32_t)dfs << 16) | ((uint32_t)cfs << 12) | ((uint32_t)srl << 11) | ((uint32_t)slv_oe << 10) | ((uint32_t)tmod << 8) | ((uint32_t)scpol << 7) | ((uint32_t)scph << 6) | ((uint32_t)frf << 4));
}

__STATIC_INLINE void qspi_ctrlr0_unpack(uint8_t* spi_frf, uint8_t* dfs, uint8_t* cfs, uint8_t* srl, uint8_t* slv_oe, uint8_t* tmod, uint8_t* scpol, uint8_t* scph, uint8_t* frf)
{
    T_QSPI_CTRLR0 localVal = hwp_qspi->ctrlr0;

    *spi_frf = localVal.bit_field.spi_frf;
    *dfs = localVal.bit_field.dfs;
    *cfs = localVal.bit_field.cfs;
    *srl = localVal.bit_field.srl;
    *slv_oe = localVal.bit_field.slv_oe;
    *tmod = localVal.bit_field.tmod;
    *scpol = localVal.bit_field.scpol;
    *scph = localVal.bit_field.scph;
    *frf = localVal.bit_field.frf;
}

__STATIC_INLINE uint8_t qspi_spi_frf_getf(void)
{
    return hwp_qspi->ctrlr0.bit_field.spi_frf;
}

__STATIC_INLINE void qspi_spi_frf_setf(uint8_t spi_frf)
{
    hwp_qspi->ctrlr0.bit_field.spi_frf = spi_frf;
}

__STATIC_INLINE uint8_t qspi_dfs_getf(void)
{
    return hwp_qspi->ctrlr0.bit_field.dfs;
}

__STATIC_INLINE void qspi_dfs_setf(uint8_t dfs)
{
    hwp_qspi->ctrlr0.bit_field.dfs = dfs;
}

__STATIC_INLINE uint8_t qspi_cfs_getf(void)
{
    return hwp_qspi->ctrlr0.bit_field.cfs;
}

__STATIC_INLINE void qspi_cfs_setf(uint8_t cfs)
{
    hwp_qspi->ctrlr0.bit_field.cfs = cfs;
}

__STATIC_INLINE uint8_t qspi_srl_getf(void)
{
    return hwp_qspi->ctrlr0.bit_field.srl;
}

__STATIC_INLINE void qspi_srl_setf(uint8_t srl)
{
    hwp_qspi->ctrlr0.bit_field.srl = srl;
}

__STATIC_INLINE uint8_t qspi_slv_oe_getf(void)
{
    return hwp_qspi->ctrlr0.bit_field.slv_oe;
}

__STATIC_INLINE void qspi_slv_oe_setf(uint8_t slv_oe)
{
    hwp_qspi->ctrlr0.bit_field.slv_oe = slv_oe;
}

__STATIC_INLINE uint8_t qspi_tmod_getf(void)
{
    return hwp_qspi->ctrlr0.bit_field.tmod;
}

__STATIC_INLINE void qspi_tmod_setf(uint8_t tmod)
{
    hwp_qspi->ctrlr0.bit_field.tmod = tmod;
}

__STATIC_INLINE uint8_t qspi_scpol_getf(void)
{
    return hwp_qspi->ctrlr0.bit_field.scpol;
}

__STATIC_INLINE void qspi_scpol_setf(uint8_t scpol)
{
    hwp_qspi->ctrlr0.bit_field.scpol = scpol;
}

__STATIC_INLINE uint8_t qspi_scph_getf(void)
{
    return hwp_qspi->ctrlr0.bit_field.scph;
}

__STATIC_INLINE void qspi_scph_setf(uint8_t scph)
{
    hwp_qspi->ctrlr0.bit_field.scph = scph;
}

__STATIC_INLINE uint8_t qspi_frf_getf(void)
{
    return hwp_qspi->ctrlr0.bit_field.frf;
}

__STATIC_INLINE void qspi_frf_setf(uint8_t frf)
{
    hwp_qspi->ctrlr0.bit_field.frf = frf;
}

__STATIC_INLINE uint32_t qspi_ctrlr1_get(void)
{
    return hwp_qspi->ctrlr1.val;
}

__STATIC_INLINE void qspi_ctrlr1_set(uint32_t value)
{
    hwp_qspi->ctrlr1.val = value;
}

__STATIC_INLINE void qspi_ctrlr1_pack(uint16_t ndf)
{
    hwp_qspi->ctrlr1.val = (((uint32_t)ndf << 0));
}

__STATIC_INLINE void qspi_ctrlr1_unpack(uint16_t* ndf)
{
    T_QSPI_CTRLR1 localVal = hwp_qspi->ctrlr1;

    *ndf = localVal.bit_field.ndf;
}

__STATIC_INLINE uint16_t qspi_ndf_getf(void)
{
    return hwp_qspi->ctrlr1.bit_field.ndf;
}

__STATIC_INLINE void qspi_ndf_setf(uint16_t ndf)
{
    hwp_qspi->ctrlr1.bit_field.ndf = ndf;
}

__STATIC_INLINE uint32_t qspi_ssienr_get(void)
{
    return hwp_qspi->ssienr.val;
}

__STATIC_INLINE void qspi_ssienr_set(uint32_t value)
{
    hwp_qspi->ssienr.val = value;
}

__STATIC_INLINE void qspi_ssienr_pack(uint8_t ssi_en)
{
    hwp_qspi->ssienr.val = (((uint32_t)ssi_en << 0));
}

__STATIC_INLINE void qspi_ssienr_unpack(uint8_t* ssi_en)
{
    T_QSPI_SSIENR localVal = hwp_qspi->ssienr;

    *ssi_en = localVal.bit_field.ssi_en;
}

__STATIC_INLINE uint8_t qspi_ssi_en_getf(void)
{
    return hwp_qspi->ssienr.bit_field.ssi_en;
}

__STATIC_INLINE void qspi_ssi_en_setf(uint8_t ssi_en)
{
    hwp_qspi->ssienr.bit_field.ssi_en = ssi_en;
}

__STATIC_INLINE uint32_t qspi_mwcr_get(void)
{
    return hwp_qspi->mwcr.val;
}

__STATIC_INLINE void qspi_mwcr_set(uint32_t value)
{
    hwp_qspi->mwcr.val = value;
}

__STATIC_INLINE void qspi_mwcr_pack(uint8_t mhs, uint8_t mod, uint8_t mwmod)
{
    hwp_qspi->mwcr.val = (((uint32_t)mhs << 2) | ((uint32_t)mod << 1) | ((uint32_t)mwmod << 0));
}

__STATIC_INLINE void qspi_mwcr_unpack(uint8_t* mhs, uint8_t* mod, uint8_t* mwmod)
{
    T_QSPI_MWCR localVal = hwp_qspi->mwcr;

    *mhs = localVal.bit_field.mhs;
    *mod = localVal.bit_field.mod;
    *mwmod = localVal.bit_field.mwmod;
}

__STATIC_INLINE uint8_t qspi_mhs_getf(void)
{
    return hwp_qspi->mwcr.bit_field.mhs;
}

__STATIC_INLINE void qspi_mhs_setf(uint8_t mhs)
{
    hwp_qspi->mwcr.bit_field.mhs = mhs;
}

__STATIC_INLINE uint8_t qspi_mod_getf(void)
{
    return hwp_qspi->mwcr.bit_field.mod;
}

__STATIC_INLINE void qspi_mod_setf(uint8_t mod)
{
    hwp_qspi->mwcr.bit_field.mod = mod;
}

__STATIC_INLINE uint8_t qspi_mwmod_getf(void)
{
    return hwp_qspi->mwcr.bit_field.mwmod;
}

__STATIC_INLINE void qspi_mwmod_setf(uint8_t mwmod)
{
    hwp_qspi->mwcr.bit_field.mwmod = mwmod;
}

__STATIC_INLINE uint32_t qspi_ser_get(void)
{
    return hwp_qspi->ser.val;
}

__STATIC_INLINE void qspi_ser_set(uint32_t value)
{
    hwp_qspi->ser.val = value;
}

__STATIC_INLINE void qspi_ser_pack(uint8_t ser)
{
    hwp_qspi->ser.val = (((uint32_t)ser << 0));
}

__STATIC_INLINE void qspi_ser_unpack(uint8_t* ser)
{
    T_QSPI_SER localVal = hwp_qspi->ser;

    *ser = localVal.bit_field.ser;
}

__STATIC_INLINE uint8_t qspi_ser_getf(void)
{
    return hwp_qspi->ser.bit_field.ser;
}

__STATIC_INLINE void qspi_ser_setf(uint8_t ser)
{
    hwp_qspi->ser.bit_field.ser = ser;
}

__STATIC_INLINE uint32_t qspi_baudr_get(void)
{
    return hwp_qspi->baudr.val;
}

__STATIC_INLINE void qspi_baudr_set(uint32_t value)
{
    hwp_qspi->baudr.val = value;
}

__STATIC_INLINE void qspi_baudr_pack(uint16_t sckdv)
{
    hwp_qspi->baudr.val = (((uint32_t)sckdv << 0));
}

__STATIC_INLINE void qspi_baudr_unpack(uint16_t* sckdv)
{
    T_QSPI_BAUDR localVal = hwp_qspi->baudr;

    *sckdv = localVal.bit_field.sckdv;
}

__STATIC_INLINE uint16_t qspi_sckdv_getf(void)
{
    return hwp_qspi->baudr.bit_field.sckdv;
}

__STATIC_INLINE void qspi_sckdv_setf(uint16_t sckdv)
{
    hwp_qspi->baudr.bit_field.sckdv = sckdv;
}

__STATIC_INLINE uint32_t qspi_txftlr_get(void)
{
    return hwp_qspi->txftlr.val;
}

__STATIC_INLINE void qspi_txftlr_set(uint32_t value)
{
    hwp_qspi->txftlr.val = value;
}

__STATIC_INLINE void qspi_txftlr_pack(uint8_t tft)
{
    hwp_qspi->txftlr.val = (((uint32_t)tft << 0));
}

__STATIC_INLINE void qspi_txftlr_unpack(uint8_t* tft)
{
    T_QSPI_TXFTLR localVal = hwp_qspi->txftlr;

    *tft = localVal.bit_field.tft;
}

__STATIC_INLINE uint8_t qspi_tft_getf(void)
{
    return hwp_qspi->txftlr.bit_field.tft;
}

__STATIC_INLINE void qspi_tft_setf(uint8_t tft)
{
    hwp_qspi->txftlr.bit_field.tft = tft;
}

__STATIC_INLINE uint32_t qspi_rxftlr_get(void)
{
    return hwp_qspi->rxftlr.val;
}

__STATIC_INLINE void qspi_rxftlr_set(uint32_t value)
{
    hwp_qspi->rxftlr.val = value;
}

__STATIC_INLINE void qspi_rxftlr_pack(uint8_t rft)
{
    hwp_qspi->rxftlr.val = (((uint32_t)rft << 0));
}

__STATIC_INLINE void qspi_rxftlr_unpack(uint8_t* rft)
{
    T_QSPI_RXFTLR localVal = hwp_qspi->rxftlr;

    *rft = localVal.bit_field.rft;
}

__STATIC_INLINE uint8_t qspi_rft_getf(void)
{
    return hwp_qspi->rxftlr.bit_field.rft;
}

__STATIC_INLINE void qspi_rft_setf(uint8_t rft)
{
    hwp_qspi->rxftlr.bit_field.rft = rft;
}

__STATIC_INLINE uint32_t qspi_txflr_get(void)
{
    return hwp_qspi->txflr.val;
}

__STATIC_INLINE void qspi_txflr_unpack(uint8_t* txtfl)
{
    T_QSPI_TXFLR localVal = hwp_qspi->txflr;

    *txtfl = localVal.bit_field.txtfl;
}

__STATIC_INLINE uint8_t qspi_txtfl_getf(void)
{
    return hwp_qspi->txflr.bit_field.txtfl;
}

__STATIC_INLINE uint32_t qspi_rxflr_get(void)
{
    return hwp_qspi->rxflr.val;
}

__STATIC_INLINE void qspi_rxflr_unpack(uint8_t* rxtfl)
{
    T_QSPI_RXFLR localVal = hwp_qspi->rxflr;

    *rxtfl = localVal.bit_field.rxtfl;
}

__STATIC_INLINE uint8_t qspi_rxtfl_getf(void)
{
    return hwp_qspi->rxflr.bit_field.rxtfl;
}

__STATIC_INLINE uint32_t qspi_sr_get(void)
{
    return hwp_qspi->sr.val;
}

__STATIC_INLINE void qspi_sr_unpack(uint8_t* dcol, uint8_t* txe, uint8_t* rff, uint8_t* rfne, uint8_t* tfe, uint8_t* tfnf, uint8_t* busy)
{
    T_QSPI_SR localVal = hwp_qspi->sr;

    *dcol = localVal.bit_field.dcol;
    *txe = localVal.bit_field.txe;
    *rff = localVal.bit_field.rff;
    *rfne = localVal.bit_field.rfne;
    *tfe = localVal.bit_field.tfe;
    *tfnf = localVal.bit_field.tfnf;
    *busy = localVal.bit_field.busy;
}

__STATIC_INLINE uint8_t qspi_dcol_getf(void)
{
    return hwp_qspi->sr.bit_field.dcol;
}

__STATIC_INLINE uint8_t qspi_txe_getf(void)
{
    return hwp_qspi->sr.bit_field.txe;
}

__STATIC_INLINE uint8_t qspi_rff_getf(void)
{
    return hwp_qspi->sr.bit_field.rff;
}

__STATIC_INLINE uint8_t qspi_rfne_getf(void)
{
    return hwp_qspi->sr.bit_field.rfne;
}

__STATIC_INLINE uint8_t qspi_tfe_getf(void)
{
    return hwp_qspi->sr.bit_field.tfe;
}

__STATIC_INLINE uint8_t qspi_tfnf_getf(void)
{
    return hwp_qspi->sr.bit_field.tfnf;
}

__STATIC_INLINE uint8_t qspi_busy_getf(void)
{
    return hwp_qspi->sr.bit_field.busy;
}

__STATIC_INLINE uint32_t qspi_imr_get(void)
{
    return hwp_qspi->imr.val;
}

__STATIC_INLINE void qspi_imr_set(uint32_t value)
{
    hwp_qspi->imr.val = value;
}

__STATIC_INLINE void qspi_imr_pack(uint8_t mstim, uint8_t rxfim, uint8_t rxoim, uint8_t rxuim, uint8_t txoim, uint8_t txeim)
{
    hwp_qspi->imr.val = (((uint32_t)mstim << 5) | ((uint32_t)rxfim << 4) | ((uint32_t)rxoim << 3) | ((uint32_t)rxuim << 2) | ((uint32_t)txoim << 1) | ((uint32_t)txeim << 0));
}

__STATIC_INLINE void qspi_imr_unpack(uint8_t* mstim, uint8_t* rxfim, uint8_t* rxoim, uint8_t* rxuim, uint8_t* txoim, uint8_t* txeim)
{
    T_QSPI_IMR localVal = hwp_qspi->imr;

    *mstim = localVal.bit_field.mstim;
    *rxfim = localVal.bit_field.rxfim;
    *rxoim = localVal.bit_field.rxoim;
    *rxuim = localVal.bit_field.rxuim;
    *txoim = localVal.bit_field.txoim;
    *txeim = localVal.bit_field.txeim;
}

__STATIC_INLINE uint8_t qspi_mstim_getf(void)
{
    return hwp_qspi->imr.bit_field.mstim;
}

__STATIC_INLINE void qspi_mstim_setf(uint8_t mstim)
{
    hwp_qspi->imr.bit_field.mstim = mstim;
}

__STATIC_INLINE uint8_t qspi_rxfim_getf(void)
{
    return hwp_qspi->imr.bit_field.rxfim;
}

__STATIC_INLINE void qspi_rxfim_setf(uint8_t rxfim)
{
    hwp_qspi->imr.bit_field.rxfim = rxfim;
}

__STATIC_INLINE uint8_t qspi_rxoim_getf(void)
{
    return hwp_qspi->imr.bit_field.rxoim;
}

__STATIC_INLINE void qspi_rxoim_setf(uint8_t rxoim)
{
    hwp_qspi->imr.bit_field.rxoim = rxoim;
}

__STATIC_INLINE uint8_t qspi_rxuim_getf(void)
{
    return hwp_qspi->imr.bit_field.rxuim;
}

__STATIC_INLINE void qspi_rxuim_setf(uint8_t rxuim)
{
    hwp_qspi->imr.bit_field.rxuim = rxuim;
}

__STATIC_INLINE uint8_t qspi_txoim_getf(void)
{
    return hwp_qspi->imr.bit_field.txoim;
}

__STATIC_INLINE void qspi_txoim_setf(uint8_t txoim)
{
    hwp_qspi->imr.bit_field.txoim = txoim;
}

__STATIC_INLINE uint8_t qspi_txeim_getf(void)
{
    return hwp_qspi->imr.bit_field.txeim;
}

__STATIC_INLINE void qspi_txeim_setf(uint8_t txeim)
{
    hwp_qspi->imr.bit_field.txeim = txeim;
}

__STATIC_INLINE uint32_t qspi_isr_get(void)
{
    return hwp_qspi->isr.val;
}

__STATIC_INLINE void qspi_isr_unpack(uint8_t* mstis, uint8_t* rxfis, uint8_t* rxois, uint8_t* rxuis, uint8_t* txois, uint8_t* txeis)
{
    T_QSPI_ISR localVal = hwp_qspi->isr;

    *mstis = localVal.bit_field.mstis;
    *rxfis = localVal.bit_field.rxfis;
    *rxois = localVal.bit_field.rxois;
    *rxuis = localVal.bit_field.rxuis;
    *txois = localVal.bit_field.txois;
    *txeis = localVal.bit_field.txeis;
}

__STATIC_INLINE uint8_t qspi_mstis_getf(void)
{
    return hwp_qspi->isr.bit_field.mstis;
}

__STATIC_INLINE uint8_t qspi_rxfis_getf(void)
{
    return hwp_qspi->isr.bit_field.rxfis;
}

__STATIC_INLINE uint8_t qspi_rxois_getf(void)
{
    return hwp_qspi->isr.bit_field.rxois;
}

__STATIC_INLINE uint8_t qspi_rxuis_getf(void)
{
    return hwp_qspi->isr.bit_field.rxuis;
}

__STATIC_INLINE uint8_t qspi_txois_getf(void)
{
    return hwp_qspi->isr.bit_field.txois;
}

__STATIC_INLINE uint8_t qspi_txeis_getf(void)
{
    return hwp_qspi->isr.bit_field.txeis;
}

__STATIC_INLINE uint32_t qspi_risr_get(void)
{
    return hwp_qspi->risr.val;
}

__STATIC_INLINE void qspi_risr_unpack(uint8_t* mstir, uint8_t* rxfir, uint8_t* rxoir, uint8_t* rxuir, uint8_t* txoir, uint8_t* txeir)
{
    T_QSPI_RISR localVal = hwp_qspi->risr;

    *mstir = localVal.bit_field.mstir;
    *rxfir = localVal.bit_field.rxfir;
    *rxoir = localVal.bit_field.rxoir;
    *rxuir = localVal.bit_field.rxuir;
    *txoir = localVal.bit_field.txoir;
    *txeir = localVal.bit_field.txeir;
}

__STATIC_INLINE uint8_t qspi_mstir_getf(void)
{
    return hwp_qspi->risr.bit_field.mstir;
}

__STATIC_INLINE uint8_t qspi_rxfir_getf(void)
{
    return hwp_qspi->risr.bit_field.rxfir;
}

__STATIC_INLINE uint8_t qspi_rxoir_getf(void)
{
    return hwp_qspi->risr.bit_field.rxoir;
}

__STATIC_INLINE uint8_t qspi_rxuir_getf(void)
{
    return hwp_qspi->risr.bit_field.rxuir;
}

__STATIC_INLINE uint8_t qspi_txoir_getf(void)
{
    return hwp_qspi->risr.bit_field.txoir;
}

__STATIC_INLINE uint8_t qspi_txeir_getf(void)
{
    return hwp_qspi->risr.bit_field.txeir;
}

__STATIC_INLINE uint32_t qspi_txoicr_get(void)
{
    return hwp_qspi->txoicr.val;
}

__STATIC_INLINE void qspi_txoicr_unpack(uint8_t* txoicr)
{
    T_QSPI_TXOICR localVal = hwp_qspi->txoicr;

    *txoicr = localVal.bit_field.txoicr;
}

__STATIC_INLINE uint8_t qspi_txoicr_getf(void)
{
    return hwp_qspi->txoicr.bit_field.txoicr;
}

__STATIC_INLINE uint32_t qspi_rxoicr_get(void)
{
    return hwp_qspi->rxoicr.val;
}

__STATIC_INLINE void qspi_rxoicr_unpack(uint8_t* rxoicr)
{
    T_QSPI_RXOICR localVal = hwp_qspi->rxoicr;

    *rxoicr = localVal.bit_field.rxoicr;
}

__STATIC_INLINE uint8_t qspi_rxoicr_getf(void)
{
    return hwp_qspi->rxoicr.bit_field.rxoicr;
}

__STATIC_INLINE uint32_t qspi_rxuicr_get(void)
{
    return hwp_qspi->rxuicr.val;
}

__STATIC_INLINE void qspi_rxuicr_unpack(uint8_t* rxuicr)
{
    T_QSPI_RXUICR localVal = hwp_qspi->rxuicr;

    *rxuicr = localVal.bit_field.rxuicr;
}

__STATIC_INLINE uint8_t qspi_rxuicr_getf(void)
{
    return hwp_qspi->rxuicr.bit_field.rxuicr;
}

__STATIC_INLINE uint32_t qspi_msticr_get(void)
{
    return hwp_qspi->msticr.val;
}

__STATIC_INLINE void qspi_msticr_unpack(uint8_t* msticr)
{
    T_QSPI_MSTICR localVal = hwp_qspi->msticr;

    *msticr = localVal.bit_field.msticr;
}

__STATIC_INLINE uint8_t qspi_msticr_getf(void)
{
    return hwp_qspi->msticr.bit_field.msticr;
}

__STATIC_INLINE uint32_t qspi_icr_get(void)
{
    return hwp_qspi->icr.val;
}

__STATIC_INLINE void qspi_icr_unpack(uint8_t* icr)
{
    T_QSPI_ICR localVal = hwp_qspi->icr;

    *icr = localVal.bit_field.icr;
}

__STATIC_INLINE uint8_t qspi_icr_getf(void)
{
    return hwp_qspi->icr.bit_field.icr;
}

__STATIC_INLINE uint32_t qspi_dmacr_get(void)
{
    return hwp_qspi->dmacr.val;
}

__STATIC_INLINE void qspi_dmacr_set(uint32_t value)
{
    hwp_qspi->dmacr.val = value;
}

__STATIC_INLINE void qspi_dmacr_pack(uint8_t tdmae, uint8_t rdmae)
{
    hwp_qspi->dmacr.val = (((uint32_t)tdmae << 1) | ((uint32_t)rdmae << 0));
}

__STATIC_INLINE void qspi_dmacr_unpack(uint8_t* tdmae, uint8_t* rdmae)
{
    T_QSPI_DMACR localVal = hwp_qspi->dmacr;

    *tdmae = localVal.bit_field.tdmae;
    *rdmae = localVal.bit_field.rdmae;
}

__STATIC_INLINE uint8_t qspi_tdmae_getf(void)
{
    return hwp_qspi->dmacr.bit_field.tdmae;
}

__STATIC_INLINE void qspi_tdmae_setf(uint8_t tdmae)
{
    hwp_qspi->dmacr.bit_field.tdmae = tdmae;
}

__STATIC_INLINE uint8_t qspi_rdmae_getf(void)
{
    return hwp_qspi->dmacr.bit_field.rdmae;
}

__STATIC_INLINE void qspi_rdmae_setf(uint8_t rdmae)
{
    hwp_qspi->dmacr.bit_field.rdmae = rdmae;
}

__STATIC_INLINE uint32_t qspi_dmatdlr_get(void)
{
    return hwp_qspi->dmatdlr.val;
}

__STATIC_INLINE void qspi_dmatdlr_set(uint32_t value)
{
    hwp_qspi->dmatdlr.val = value;
}

__STATIC_INLINE void qspi_dmatdlr_pack(uint8_t dmatdl)
{
    hwp_qspi->dmatdlr.val = (((uint32_t)dmatdl << 0));
}

__STATIC_INLINE void qspi_dmatdlr_unpack(uint8_t* dmatdl)
{
    T_QSPI_DMATDLR localVal = hwp_qspi->dmatdlr;

    *dmatdl = localVal.bit_field.dmatdl;
}

__STATIC_INLINE uint8_t qspi_dmatdl_getf(void)
{
    return hwp_qspi->dmatdlr.bit_field.dmatdl;
}

__STATIC_INLINE void qspi_dmatdl_setf(uint8_t dmatdl)
{
    hwp_qspi->dmatdlr.bit_field.dmatdl = dmatdl;
}

__STATIC_INLINE uint32_t qspi_dmardlr_get(void)
{
    return hwp_qspi->dmardlr.val;
}

__STATIC_INLINE void qspi_dmardlr_set(uint32_t value)
{
    hwp_qspi->dmardlr.val = value;
}

__STATIC_INLINE void qspi_dmardlr_pack(uint8_t dmardl)
{
    hwp_qspi->dmardlr.val = (((uint32_t)dmardl << 0));
}

__STATIC_INLINE void qspi_dmardlr_unpack(uint8_t* dmardl)
{
    T_QSPI_DMARDLR localVal = hwp_qspi->dmardlr;

    *dmardl = localVal.bit_field.dmardl;
}

__STATIC_INLINE uint8_t qspi_dmardl_getf(void)
{
    return hwp_qspi->dmardlr.bit_field.dmardl;
}

__STATIC_INLINE void qspi_dmardl_setf(uint8_t dmardl)
{
    hwp_qspi->dmardlr.bit_field.dmardl = dmardl;
}

__STATIC_INLINE uint32_t qspi_idr_get(void)
{
    return hwp_qspi->idr.val;
}

__STATIC_INLINE void qspi_idr_unpack(uint32_t* idcode)
{
    T_QSPI_IDR localVal = hwp_qspi->idr;

    *idcode = localVal.bit_field.idcode;
}

__STATIC_INLINE uint32_t qspi_idcode_getf(void)
{
    return hwp_qspi->idr.bit_field.idcode;
}

__STATIC_INLINE uint32_t qspi_ssi_comp_version_get(void)
{
    return hwp_qspi->ssi_comp_version.val;
}

__STATIC_INLINE void qspi_ssi_comp_version_unpack(uint32_t* ssi_comp_version)
{
    T_QSPI_SSI_COMP_VERSION localVal = hwp_qspi->ssi_comp_version;

    *ssi_comp_version = localVal.bit_field.ssi_comp_version;
}

__STATIC_INLINE uint32_t qspi_ssi_comp_version_getf(void)
{
    return hwp_qspi->ssi_comp_version.bit_field.ssi_comp_version;
}

__STATIC_INLINE uint32_t qspi_dr_get(void)
{
    return hwp_qspi->dr.val;
}

__STATIC_INLINE void qspi_dr_set(uint32_t value)
{
    hwp_qspi->dr.val = value;
}

__STATIC_INLINE void qspi_dr_pack(uint32_t dr)
{
    hwp_qspi->dr.val = (((uint32_t)dr << 0));
}

__STATIC_INLINE void qspi_dr_unpack(uint32_t* dr)
{
    T_QSPI_DR localVal = hwp_qspi->dr;

    *dr = localVal.bit_field.dr;
}

__STATIC_INLINE uint32_t qspi_dr_getf(void)
{
    return hwp_qspi->dr.bit_field.dr;
}

__STATIC_INLINE void qspi_dr_setf(uint32_t dr)
{
    hwp_qspi->dr.bit_field.dr = dr;
}

__STATIC_INLINE uint32_t qspi_dr_reversed_get(void)
{
    return hwp_qspi->dr_reversed.val;
}

__STATIC_INLINE void qspi_dr_reversed_set(uint32_t value)
{
    hwp_qspi->dr_reversed.val = value;
}

__STATIC_INLINE void qspi_dr_reversed_pack(uint32_t dr_reversed)
{
    hwp_qspi->dr_reversed.val = (((uint32_t)dr_reversed << 0));
}

__STATIC_INLINE void qspi_dr_reversed_unpack(uint32_t* dr_reversed)
{
    T_QSPI_DR_REVERSED localVal = hwp_qspi->dr_reversed;

    *dr_reversed = localVal.bit_field.dr_reversed;
}

__STATIC_INLINE uint32_t qspi_dr_reversed_getf(void)
{
    return hwp_qspi->dr_reversed.bit_field.dr_reversed;
}

__STATIC_INLINE void qspi_dr_reversed_setf(uint32_t dr_reversed)
{
    hwp_qspi->dr_reversed.bit_field.dr_reversed = dr_reversed;
}

__STATIC_INLINE uint32_t qspi_rxsample_dly_get(void)
{
    return hwp_qspi->rxsample_dly.val;
}

__STATIC_INLINE void qspi_rxsample_dly_set(uint32_t value)
{
    hwp_qspi->rxsample_dly.val = value;
}

__STATIC_INLINE void qspi_rxsample_dly_pack(uint8_t rsd)
{
    hwp_qspi->rxsample_dly.val = (((uint32_t)rsd << 0));
}

__STATIC_INLINE void qspi_rxsample_dly_unpack(uint8_t* rsd)
{
    T_QSPI_RXSAMPLE_DLY localVal = hwp_qspi->rxsample_dly;

    *rsd = localVal.bit_field.rsd;
}

__STATIC_INLINE uint8_t qspi_rsd_getf(void)
{
    return hwp_qspi->rxsample_dly.bit_field.rsd;
}

__STATIC_INLINE void qspi_rsd_setf(uint8_t rsd)
{
    hwp_qspi->rxsample_dly.bit_field.rsd = rsd;
}

__STATIC_INLINE uint32_t qspi_spi_ctrlr0_get(void)
{
    return hwp_qspi->spi_ctrlr0.val;
}

__STATIC_INLINE void qspi_spi_ctrlr0_set(uint32_t value)
{
    hwp_qspi->spi_ctrlr0.val = value;
}

__STATIC_INLINE void qspi_spi_ctrlr0_pack(uint8_t wait_cycles, uint8_t inst_l, uint8_t addr_l, uint8_t trans_type)
{
    hwp_qspi->spi_ctrlr0.val = (((uint32_t)wait_cycles << 11) | ((uint32_t)inst_l << 8) | ((uint32_t)addr_l << 2) | ((uint32_t)trans_type << 0));
}

__STATIC_INLINE void qspi_spi_ctrlr0_unpack(uint8_t* wait_cycles, uint8_t* inst_l, uint8_t* addr_l, uint8_t* trans_type)
{
    T_QSPI_SPI_CTRLR0 localVal = hwp_qspi->spi_ctrlr0;

    *wait_cycles = localVal.bit_field.wait_cycles;
    *inst_l = localVal.bit_field.inst_l;
    *addr_l = localVal.bit_field.addr_l;
    *trans_type = localVal.bit_field.trans_type;
}

__STATIC_INLINE uint8_t qspi_wait_cycles_getf(void)
{
    return hwp_qspi->spi_ctrlr0.bit_field.wait_cycles;
}

__STATIC_INLINE void qspi_wait_cycles_setf(uint8_t wait_cycles)
{
    hwp_qspi->spi_ctrlr0.bit_field.wait_cycles = wait_cycles;
}

__STATIC_INLINE uint8_t qspi_inst_l_getf(void)
{
    return hwp_qspi->spi_ctrlr0.bit_field.inst_l;
}

__STATIC_INLINE void qspi_inst_l_setf(uint8_t inst_l)
{
    hwp_qspi->spi_ctrlr0.bit_field.inst_l = inst_l;
}

__STATIC_INLINE uint8_t qspi_addr_l_getf(void)
{
    return hwp_qspi->spi_ctrlr0.bit_field.addr_l;
}

__STATIC_INLINE void qspi_addr_l_setf(uint8_t addr_l)
{
    hwp_qspi->spi_ctrlr0.bit_field.addr_l = addr_l;
}

__STATIC_INLINE uint8_t qspi_trans_type_getf(void)
{
    return hwp_qspi->spi_ctrlr0.bit_field.trans_type;
}

__STATIC_INLINE void qspi_trans_type_setf(uint8_t trans_type)
{
    hwp_qspi->spi_ctrlr0.bit_field.trans_type = trans_type;
}
#endif

