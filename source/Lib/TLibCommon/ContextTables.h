/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.  
 *
 * Copyright (c) 2010-2014, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     ContextTables.h
    \brief    Defines constants and tables for SBAC
    \todo     number of context models is not matched to actual use, should be fixed
*/

#ifndef __CONTEXTTABLES__
#define __CONTEXTTABLES__

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define MAX_NUM_CTX_MOD             512       ///< maximum number of supported contexts

#define NUM_SPLIT_FLAG_CTX            3       ///< number of context models for split flag
#define NUM_SKIP_FLAG_CTX             3       ///< number of context models for skip flag

#define NUM_MERGE_FLAG_EXT_CTX        1       ///< number of context models for merge flag of merge extended
#if IT_GT
#define NUM_GT_FLAG_EXT_CTX		  1
#endif
#define NUM_MERGE_IDX_EXT_CTX         1       ///< number of context models for merge index of merge extended

#define NUM_PART_SIZE_CTX             4       ///< number of context models for partition size
#define NUM_PRED_MODE_CTX             1       ///< number of context models for prediction mode

#define NUM_ADI_CTX                   1       ///< number of context models for intra prediction

#define NUM_CHROMA_PRED_CTX           2       ///< number of context models for intra prediction (chroma)
#define NUM_INTER_DIR_CTX             5       ///< number of context models for inter prediction direction
#define NUM_MV_RES_CTX                2       ///< number of context models for motion vector difference
#if IT_GT
#define NUM_GT_RES_CTX                2       ///< number of context models for gt
#endif

#define NUM_REF_NO_CTX                2       ///< number of context models for reference index
#define NUM_TRANS_SUBDIV_FLAG_CTX     3       ///< number of context models for transform subdivision flags
#define NUM_QT_CBF_CTX                4       ///< number of context models for QT CBF
#define NUM_QT_ROOT_CBF_CTX           1       ///< number of context models for QT ROOT CBF
#define NUM_DELTA_QP_CTX              3       ///< number of context models for dQP

#define NUM_SIG_CG_FLAG_CTX           2       ///< number of context models for MULTI_LEVEL_SIGNIFICANCE

#define NUM_SIG_FLAG_CTX              42      ///< number of context models for sig flag
#define NUM_SIG_FLAG_CTX_LUMA         27      ///< number of context models for luma sig flag
#define NUM_SIG_FLAG_CTX_CHROMA       15      ///< number of context models for chroma sig flag

#define NUM_CTX_LAST_FLAG_XY          15      ///< number of context models for last coefficient position

#define NUM_ONE_FLAG_CTX              24      ///< number of context models for greater than 1 flag
#define NUM_ONE_FLAG_CTX_LUMA         16      ///< number of context models for greater than 1 flag of luma
#define NUM_ONE_FLAG_CTX_CHROMA        8      ///< number of context models for greater than 1 flag of chroma
#define NUM_ABS_FLAG_CTX               6      ///< number of context models for greater than 2 flag
#define NUM_ABS_FLAG_CTX_LUMA          4      ///< number of context models for greater than 2 flag of luma
#define NUM_ABS_FLAG_CTX_CHROMA        2      ///< number of context models for greater than 2 flag of chroma

#define NUM_MVP_IDX_CTX               1       ///< number of context models for MVP index

#define NUM_SAO_MERGE_FLAG_CTX        1       ///< number of context models for SAO merge flags
#define NUM_SAO_TYPE_IDX_CTX          1       ///< number of context models for SAO type index

#define NUM_TRANSFORMSKIP_FLAG_CTX    1       ///< number of context models for transform skipping 
#define NUM_CU_TRANSQUANT_BYPASS_FLAG_CTX  1 
#define CNU                          154      ///< dummy initialization value for unused context models 'Context model Not Used'

// ====================================================================================================================
// Tables
// ====================================================================================================================

// initial probability for cu_transquant_bypass flag
static const UChar
#if IT_HOLOSS
INIT_CU_TRANSQUANT_BYPASS_FLAG[5][NUM_CU_TRANSQUANT_BYPASS_FLAG_CTX] =
#else
INIT_CU_TRANSQUANT_BYPASS_FLAG[3][NUM_CU_TRANSQUANT_BYPASS_FLAG_CTX] =
#endif
{
  { 154 }, 
  { 154 }, 
  { 154 },
#if IT_HOLOSS
  { 154 },
  { 154 },
#endif 
};

// initial probability for split flag
static const UChar
#if IT_HOLOSS
INIT_SPLIT_FLAG[5][NUM_SPLIT_FLAG_CTX] =  
#else
INIT_SPLIT_FLAG[3][NUM_SPLIT_FLAG_CTX] =  
#endif
{
  { 107,  139,  126, },
  { 107,  139,  126, }, 
  { 139,  141,  157, },
#if IT_HOLOSS
  { 107,  139,  126, },
  { 107,  139,  126, }, 
#endif 
};

static const UChar
#if IT_HOLOSS
INIT_SKIP_FLAG[5][NUM_SKIP_FLAG_CTX] = 
#else
INIT_SKIP_FLAG[3][NUM_SKIP_FLAG_CTX] =  
#endif
{
  { 197,  185,  201, }, 
  { 197,  185,  201, }, 
  { CNU,  CNU,  CNU, },
#if IT_HOLOSS
  { 197,  185,  201, },
  { 197,  185,  201, }, 
#endif 
};

static const UChar
#if IT_HOLOSS
INIT_MERGE_FLAG_EXT[5][NUM_MERGE_FLAG_EXT_CTX] =
#else
INIT_MERGE_FLAG_EXT[3][NUM_MERGE_FLAG_EXT_CTX] = 
#endif
{
  { 154, }, 
  { 110, }, 
  { CNU, },
#if IT_HOLOSS
  { 110, },
  { 110, }, 
#endif 
};

#if IT_GT
static const UChar
INIT_GT_FLAG_EXT[5][NUM_GT_FLAG_EXT_CTX] =
{
  { 154, },
  { 110, },
  { CNU, },
  { 110, },
  { 110, },
};
#endif

static const UChar
#if IT_HOLOSS
INIT_MERGE_IDX_EXT[5][NUM_MERGE_IDX_EXT_CTX] =
#else
INIT_MERGE_IDX_EXT[3][NUM_MERGE_IDX_EXT_CTX] =  
#endif
{
  { 137, }, 
  { 122, }, 
  { CNU, },
#if IT_HOLOSS
  { 122, },
  { 122, }, 
#endif 
};

static const UChar
#if IT_HOLOSS
INIT_PART_SIZE[5][NUM_PART_SIZE_CTX] =  
#else
INIT_PART_SIZE[3][NUM_PART_SIZE_CTX] =  
#endif
{
  { 154,  139,  154,  154 },
  { 154,  139,  154,  154 },
  { 184,  CNU,  CNU,  CNU },
#if IT_HOLOSS
  { 154,  139,  154,  154 },
  { 154,  139,  154,  154 }, 
#endif 
};

static const UChar
#if IT_HOLOSS
INIT_PRED_MODE[5][NUM_PRED_MODE_CTX] =
#else
INIT_PRED_MODE[3][NUM_PRED_MODE_CTX] = 
#endif
{
  { 134, }, 
  { 149, }, 
  { CNU, },
#if IT_HOLOSS
  { 149, },
  { 149, }, 
#endif 
};

static const UChar
#if IT_HOLOSS
INIT_INTRA_PRED_MODE[5][NUM_ADI_CTX] =
#else
INIT_INTRA_PRED_MODE[3][NUM_ADI_CTX] = 
#endif
{
  { 183, }, 
  { 154, }, 
  { 184, },
#if IT_HOLOSS
  { 154, },
  { 154, }, 
#endif 
};

static const UChar
#if IT_HOLOSS
INIT_CHROMA_PRED_MODE[5][NUM_CHROMA_PRED_CTX] = 
#else
INIT_CHROMA_PRED_MODE[3][NUM_CHROMA_PRED_CTX] = 
#endif
{
  { 152,  139, }, 
  { 152,  139, }, 
  {  63,  139, },
#if IT_HOLOSS
  { 152,  139, },
  { 152,  139, }, 
#endif 
};

static const UChar
#if IT_HOLOSS
INIT_INTER_DIR[5][NUM_INTER_DIR_CTX] = 
#else
INIT_INTER_DIR[3][NUM_INTER_DIR_CTX] = 
#endif
{
  {  95,   79,   63,   31,  31, }, 
  {  95,   79,   63,   31,  31, }, 
  { CNU,  CNU,  CNU,  CNU, CNU, },
#if IT_HOLOSS
  {  95,   79,   63,   31,  31, },
  {  95,   79,   63,   31,  31, }, 
#endif 
};

static const UChar
#if IT_HOLOSS
INIT_MVD[5][NUM_MV_RES_CTX] =
#else
INIT_MVD[3][NUM_MV_RES_CTX] =  
#endif
{
  { 169,  198, }, 
  { 140,  198, }, 
  { CNU,  CNU, },
#if IT_HOLOSS
  { 140,  198, },
  { 140,  198, },
#endif
};

#if IT_GT
static const UChar
INIT_GT[5][NUM_GT_RES_CTX] =
{
		{ 169,  198, },
		{ 140,  198, },
		{ CNU,  CNU, },
		{ 140,  198, },
		{ 140,  198, },
};
#endif

static const UChar
#if IT_HOLOSS
INIT_REF_PIC[5][NUM_REF_NO_CTX] =  
#else
INIT_REF_PIC[3][NUM_REF_NO_CTX] =  
#endif
{
  { 153,  153 }, 
  { 153,  153 }, 
  { CNU,  CNU }, 
#if IT_HOLOSS
  { 153,  153 },
  { 153,  153 }, 
#endif
};

static const UChar
#if IT_HOLOSS
INIT_DQP[5][NUM_DELTA_QP_CTX] =
#else
INIT_DQP[3][NUM_DELTA_QP_CTX] = 
#endif 
{
  { 154,  154,  154, }, 
  { 154,  154,  154, }, 
  { 154,  154,  154, }, 
#if IT_HOLOSS
  { 154,  154,  154, },
  { 154,  154,  154, }, 
#endif
};

static const UChar 
#if IT_HOLOSS
INIT_QT_CBF[5][2*NUM_QT_CBF_CTX] = 
#else
INIT_QT_CBF[3][2*NUM_QT_CBF_CTX] =  
#endif 
{
  { 153,  111,  CNU,  CNU,   149,   92,  167,  154 },
  { 153,  111,  CNU,  CNU,   149,  107,  167,  154 },
  { 111,  141,  CNU,  CNU,    94,  138,  182,  154 },
#if IT_HOLOSS
  { 153,  111,  CNU,  CNU,   149,  107,  167,  154 },
  { 153,  111,  CNU,  CNU,   149,  107,  167,  154 }, 
#endif 
};

static const UChar
#if IT_HOLOSS
INIT_QT_ROOT_CBF[5][NUM_QT_ROOT_CBF_CTX] =
#else
INIT_QT_ROOT_CBF[3][NUM_QT_ROOT_CBF_CTX] = 
#endif  
{
  {  79, }, 
  {  79, }, 
  { CNU, }, 
#if IT_HOLOSS
  {  79, },
  {  79, }, 
#endif
};

static const UChar
#if IT_HOLOSS
INIT_LAST[5][2*NUM_CTX_LAST_FLAG_XY] = 
#else
INIT_LAST[3][2*NUM_CTX_LAST_FLAG_XY] =  
#endif  
{
  { 125,  110,  124,  110,   95,   94,  125,  111,  111,   79,  125,  126,  111,  111,   79,
    108,  123,   93,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU, 
  }, 
  { 125,  110,   94,  110,   95,   79,  125,  111,  110,   78,  110,  111,  111,   95,   94,
    108,  123,  108,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,
  }, 
  { 110,  110,  124,  125,  140,  153,  125,  127,  140,  109,  111,  143,  127,  111,   79, 
    108,  123,   63,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU, 
  }, 
#if IT_HOLOSS
  { 125,  110,   94,  110,   95,   79,  125,  111,  110,   78,  110,  111,  111,   95,   94,
    108,  123,  108,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,
  },
  { 125,  110,   94,  110,   95,   79,  125,  111,  110,   78,  110,  111,  111,   95,   94,
    108,  123,  108,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,
  }, 
#endif
};

static const UChar
#if IT_HOLOSS
INIT_SIG_CG_FLAG[5][2 * NUM_SIG_CG_FLAG_CTX] = 
#else
INIT_SIG_CG_FLAG[3][2 * NUM_SIG_CG_FLAG_CTX] =  
#endif  
{
  { 121,  140,  
    61,  154, 
  }, 
  { 121,  140, 
    61,  154, 
  }, 
  {  91,  171,  
    134,  141, 
  },
#if IT_HOLOSS
  { 121,  140, 
    61,  154, 
  },
  { 121,  140, 
    61,  154, 
  }, 
#endif 
};

static const UChar
#if IT_HOLOSS
INIT_SIG_FLAG[5][NUM_SIG_FLAG_CTX] =
#else
INIT_SIG_FLAG[3][NUM_SIG_FLAG_CTX] = 
#endif  
{
  { 170,  154,  139,  153,  139,  123,  123,   63,  124,  166,  183,  140,  136,  153,  154,  166,  183,  140,  136,  153,  154,  166,  183,  140,  136,  153,  154,  170,  153,  138,  138,  122,  121,  122,  121,  167,  151,  183,  140,  151,  183,  140,  }, 
  { 155,  154,  139,  153,  139,  123,  123,   63,  153,  166,  183,  140,  136,  153,  154,  166,  183,  140,  136,  153,  154,  166,  183,  140,  136,  153,  154,  170,  153,  123,  123,  107,  121,  107,  121,  167,  151,  183,  140,  151,  183,  140,  }, 
  { 111,  111,  125,  110,  110,   94,  124,  108,  124,  107,  125,  141,  179,  153,  125,  107,  125,  141,  179,  153,  125,  107,  125,  141,  179,  153,  125,  140,  139,  182,  182,  152,  136,  152,  136,  153,  136,  139,  111,  136,  139,  111,  }, 
#if IT_HOLOSS
  { 155,  154,  139,  153,  139,  123,  123,   63,  153,  166,  183,  140,  136,  153,  154,  166,  183,  140,  136,  153,  154,  166,  183,  140,  136,  153,  154,  170,  153,  123,  123,  107,  121,  107,  121,  167,  151,  183,  140,  151,  183,  140,  }, 
  { 155,  154,  139,  153,  139,  123,  123,   63,  153,  166,  183,  140,  136,  153,  154,  166,  183,  140,  136,  153,  154,  166,  183,  140,  136,  153,  154,  170,  153,  123,  123,  107,  121,  107,  121,  167,  151,  183,  140,  151,  183,  140,  }, 
#endif
};

static const UChar
#if IT_HOLOSS
INIT_ONE_FLAG[5][NUM_ONE_FLAG_CTX] = 
#else
INIT_ONE_FLAG[3][NUM_ONE_FLAG_CTX] = 
#endif  
{
  { 154,  196,  167,  167,  154,  152,  167,  182,  182,  134,  149,  136,  153,  121,  136,  122,  169,  208,  166,  167,  154,  152,  167,  182, }, 
  { 154,  196,  196,  167,  154,  152,  167,  182,  182,  134,  149,  136,  153,  121,  136,  137,  169,  194,  166,  167,  154,  167,  137,  182, }, 
  { 140,   92,  137,  138,  140,  152,  138,  139,  153,   74,  149,   92,  139,  107,  122,  152,  140,  179,  166,  182,  140,  227,  122,  197, },
#if IT_HOLOSS
  { 154,  196,  196,  167,  154,  152,  167,  182,  182,  134,  149,  136,  153,  121,  136,  137,  169,  194,  166,  167,  154,  167,  137,  182, }, 
  { 154,  196,  196,  167,  154,  152,  167,  182,  182,  134,  149,  136,  153,  121,  136,  137,  169,  194,  166,  167,  154,  167,  137,  182, }, 
#endif 
};

static const UChar
#if IT_HOLOSS
INIT_ABS_FLAG[5][NUM_ABS_FLAG_CTX] = 
#else
INIT_ABS_FLAG[3][NUM_ABS_FLAG_CTX] =  
#endif  
{
  { 107,  167,   91,  107,  107,  167, }, 
  { 107,  167,   91,  122,  107,  167, }, 
  { 138,  153,  136,  167,  152,  152, },
#if IT_HOLOSS
  { 107,  167,   91,  122,  107,  167, }, 
  { 107,  167,   91,  122,  107,  167, }, 
#endif 
};

static const UChar
#if IT_HOLOSS
INIT_MVP_IDX[5][NUM_MVP_IDX_CTX] =  
#else
INIT_MVP_IDX[3][NUM_MVP_IDX_CTX] =  
#endif  
{
  { 168 },
  { 168 },
  { CNU }, 
#if IT_HOLOSS
  { 168 },
  { 168 }, 
#endif 
};

static const UChar
#if IT_HOLOSS
INIT_SAO_MERGE_FLAG[5][NUM_SAO_MERGE_FLAG_CTX] = 
#else
INIT_SAO_MERGE_FLAG[3][NUM_SAO_MERGE_FLAG_CTX] = 
#endif  
{
  { 153,  }, 
  { 153,  }, 
  { 153,  },
#if IT_HOLOSS
  { 153,  },
  { 153,  }, 
#endif 
};

static const UChar
#if IT_HOLOSS
INIT_SAO_TYPE_IDX[5][NUM_SAO_TYPE_IDX_CTX] = 
#else
INIT_SAO_TYPE_IDX[3][NUM_SAO_TYPE_IDX_CTX] = 
#endif  
{
  { 160, },
  { 185, },
  { 200, },
#if IT_HOLOSS
  { 185, },
  { 185, }, 
#endif 
};

static const UChar
#if IT_HOLOSS
INIT_TRANS_SUBDIV_FLAG[5][NUM_TRANS_SUBDIV_FLAG_CTX] =
#else
INIT_TRANS_SUBDIV_FLAG[3][NUM_TRANS_SUBDIV_FLAG_CTX] =
#endif 
{
  { 224,  167,  122, },
  { 124,  138,   94, },
  { 153,  138,  138, },
#if IT_HOLOSS
  { 124,  138,   94, },
  { 124,  138,   94, },
#endif
};

static const UChar
#if IT_HOLOSS
INIT_TRANSFORMSKIP_FLAG[5][2*NUM_TRANSFORMSKIP_FLAG_CTX] =
#else
INIT_TRANSFORMSKIP_FLAG[3][2*NUM_TRANSFORMSKIP_FLAG_CTX] = 
#endif 
{
  { 139,  139}, 
  { 139,  139}, 
  { 139,  139},
#if IT_HOLOSS
  { 139,  139},
  { 139,  139}, 
#endif
};
//! \}


#endif
