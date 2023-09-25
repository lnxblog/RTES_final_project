/*
 * g711.c
 *
 *  Created on: May 3, 2023
 *      Author: hemanth
 */

#define SIGN_BIT    (0x80)      /* Sign bit for a A-law byte. */
#define QUANT_MASK  (0xf)       /* Quantization field mask. */
#define NSEGS       (8)     /* Number of A-law segments. */
#define SEG_SHIFT   (4)     /* Left shift for segment number. */
#define SEG_MASK    (0x70)      /* Segment field mask. */
#define BIAS        (0x84)      /* Bias for linear code. */
#define CLIP            8159

static short seg_aend[8] = {0x1F, 0x3F, 0x7F, 0xFF,
                0x1FF, 0x3FF, 0x7FF, 0xFFF};

static short seg_uend[8] = {0x3F, 0x7F, 0xFF, 0x1FF,
                0x3FF, 0x7FF, 0xFFF, 0x1FFF};

static short search(short val,short *table,short size)
{
    short       i;

    for (i = 0; i < size; i++) {
        if (val <= *table++)
            return (i);
    }
    return (size);
}

//Performs G.711 encoding on voice data from ADC. Has to be re-entrant
unsigned char g711_encode(short pcm_val)
{
    short       mask;
    short       seg;
    unsigned char   uval;

    /* Get the sign and the magnitude of the value. */
    pcm_val = pcm_val >> 2;
    if (pcm_val < 0) {
        pcm_val = -pcm_val;
        mask = 0x7F;
    } else {
        mask = 0xFF;
    }
        if ( pcm_val > CLIP ) pcm_val = CLIP;       /* clip the magnitude */
    pcm_val += (BIAS >> 2);

    /* Convert the scaled magnitude to segment number. */
    seg = search(pcm_val, seg_uend, 8);

    /*
     * Combine the sign, segment, quantization bits;
     * and complement the code word.
     */
    if (seg >= 8)       /* out of range, return maximum value. */
        return (unsigned char) (0x7F ^ mask);
    else {
        uval = (unsigned char) (seg << 4) | ((pcm_val >> (seg + 1)) & 0xF);
        return (uval ^ mask);
    }
}
