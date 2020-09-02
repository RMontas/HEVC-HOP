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

/** \file     TComPrediction.cpp
    \brief    prediction class
*/

#include <memory.h>
#include "TComPrediction.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

TComPrediction::TComPrediction()
: m_pLumaRecBuffer(0)
, m_iLumaRecStride(0)
{
  m_piYuvExt = NULL;
}

TComPrediction::~TComPrediction()
{
  
  delete[] m_piYuvExt;

  m_acYuvPred[0].destroy();
  m_acYuvPred[1].destroy();

  m_cYuvPredTemp.destroy();

  if( m_pLumaRecBuffer )
  {
    delete [] m_pLumaRecBuffer;
  }
  
  Int i, j;
  for (i = 0; i < 4; i++)
  {
    for (j = 0; j < 4; j++)
    {
      m_filteredBlock[i][j].destroy();
    }
    m_filteredBlockTmp[i].destroy();
  }
}

Void TComPrediction::initTempBuff()
{
  if( m_piYuvExt == NULL )
  {
    Int extWidth  = MAX_CU_SIZE
#if IT_GT
    		*3
#endif
    		+ 16;
    Int extHeight = MAX_CU_SIZE
#if IT_GT
    		*3
#endif
			+ 1;
    Int i, j;
    for (i = 0; i < 4; i++)
    {
      m_filteredBlockTmp[i].create(extWidth, extHeight + 7);
      for (j = 0; j < 4; j++)
      {
        m_filteredBlock[i][j].create(extWidth, extHeight);
      }
    }
    m_iYuvExtHeight  = ((MAX_CU_SIZE + 2) << 4);
    m_iYuvExtStride = ((MAX_CU_SIZE  + 8) << 4);
    m_piYuvExt = new Int[ m_iYuvExtStride * m_iYuvExtHeight ];

    // new structure
    m_acYuvPred[0] .create( MAX_CU_SIZE, MAX_CU_SIZE );
    m_acYuvPred[1] .create( MAX_CU_SIZE, MAX_CU_SIZE );

    m_cYuvPredTemp.create( MAX_CU_SIZE, MAX_CU_SIZE );
  }

  if (m_iLumaRecStride != (MAX_CU_SIZE>>1) + 1)
  {
    m_iLumaRecStride =  (MAX_CU_SIZE>>1) + 1;
    if (!m_pLumaRecBuffer)
    {
      m_pLumaRecBuffer = new Pel[ m_iLumaRecStride * m_iLumaRecStride ];
    }
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
Pel TComPrediction::predIntraGetPredValDC( Int* pSrc, Int iSrcStride, UInt iWidth, UInt iHeight, Bool bAbove, Bool bLeft )
{
  assert(iWidth > 0 && iHeight > 0);
  Int iInd, iSum = 0;
  Pel pDcVal;

  if (bAbove)
  {
    for (iInd = 0;iInd < iWidth;iInd++)
    {
      iSum += pSrc[iInd-iSrcStride];
    }
  }
  if (bLeft)
  {
    for (iInd = 0;iInd < iHeight;iInd++)
    {
      iSum += pSrc[iInd*iSrcStride-1];
    }
  }

  if (bAbove && bLeft)
  {
    pDcVal = (iSum + iWidth) / (iWidth + iHeight);
  }
  else if (bAbove)
  {
    pDcVal = (iSum + iWidth/2) / iWidth;
  }
  else if (bLeft)
  {
    pDcVal = (iSum + iHeight/2) / iHeight;
  }
  else
  {
    pDcVal = pSrc[-1]; // Default DC value already calculated and placed in the prediction array if no neighbors are available
  }
  
  return pDcVal;
}

// Function for deriving the angular Intra predictions

/** Function for deriving the simplified angular intra predictions.
 * \param pSrc pointer to reconstructed sample array
 * \param srcStride the stride of the reconstructed sample array
 * \param rpDst reference to pointer for the prediction sample array
 * \param dstStride the stride of the prediction sample array
 * \param width the width of the block
 * \param height the height of the block
 * \param dirMode the intra prediction mode index
 * \param blkAboveAvailable boolean indication if the block above is available
 * \param blkLeftAvailable boolean indication if the block to the left is available
 *
 * This function derives the prediction samples for the angular mode based on the prediction direction indicated by
 * the prediction mode index. The prediction direction is given by the displacement of the bottom row of the block and
 * the reference row above the block in the case of vertical prediction or displacement of the rightmost column
 * of the block and reference column left from the block in the case of the horizontal prediction. The displacement
 * is signalled at 1/32 pixel accuracy. When projection of the predicted pixel falls inbetween reference samples,
 * the predicted value for the pixel is linearly interpolated from the reference samples. All reference samples are taken
 * from the extended main reference.
 */
Void TComPrediction::xPredIntraAng(Int bitDepth, Int* pSrc, Int srcStride, Pel*& rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter )
{
  Int k,l;
  Int blkSize        = width;
  Pel* pDst          = rpDst;

  // Map the mode index to main prediction direction and angle
  assert( dirMode > 0 ); //no planar
  Bool modeDC        = dirMode < 2;
  Bool modeHor       = !modeDC && (dirMode < 18);
  Bool modeVer       = !modeDC && !modeHor;
  Int intraPredAngle = modeVer ? (Int)dirMode - VER_IDX : modeHor ? -((Int)dirMode - HOR_IDX) : 0;
  Int absAng         = abs(intraPredAngle);
  Int signAng        = intraPredAngle < 0 ? -1 : 1;

  // Set bitshifts and scale the angle parameter to block size
  Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};
  Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
  Int invAngle       = invAngTable[absAng];
  absAng             = angTable[absAng];
  intraPredAngle     = signAng * absAng;

  // Do the DC prediction
  if (modeDC)
  {
    Pel dcval = predIntraGetPredValDC(pSrc, srcStride, width, height, blkAboveAvailable, blkLeftAvailable);

    for (k=0;k<blkSize;k++)
    {
      for (l=0;l<blkSize;l++)
      {
        pDst[k*dstStride+l] = dcval;
      }
    }
  }

  // Do angular predictions
  else
  {
    Pel* refMain;
    Pel* refSide;
    Pel  refAbove[2*MAX_CU_SIZE+1];
    Pel  refLeft[2*MAX_CU_SIZE+1];

    // Initialise the Main and Left reference array.
    if (intraPredAngle < 0)
    {
      for (k=0;k<blkSize+1;k++)
      {
        refAbove[k+blkSize-1] = pSrc[k-srcStride-1];
      }
      for (k=0;k<blkSize+1;k++)
      {
        refLeft[k+blkSize-1] = pSrc[(k-1)*srcStride-1];
      }
      refMain = (modeVer ? refAbove : refLeft) + (blkSize-1);
      refSide = (modeVer ? refLeft : refAbove) + (blkSize-1);

      // Extend the Main reference to the left.
      Int invAngleSum    = 128;       // rounding for (shift by 8)
      for (k=-1; k>blkSize*intraPredAngle>>5; k--)
      {
        invAngleSum += invAngle;
        refMain[k] = refSide[invAngleSum>>8];
      }
    }
    else
    {
      for (k=0;k<2*blkSize+1;k++)
      {
        refAbove[k] = pSrc[k-srcStride-1];
      }
      for (k=0;k<2*blkSize+1;k++)
      {
        refLeft[k] = pSrc[(k-1)*srcStride-1];
      }
      refMain = modeVer ? refAbove : refLeft;
      refSide = modeVer ? refLeft  : refAbove;
    }

    if (intraPredAngle == 0)
    {
      for (k=0;k<blkSize;k++)
      {
        for (l=0;l<blkSize;l++)
        {
          pDst[k*dstStride+l] = refMain[l+1];
        }
      }

      if ( bFilter )
      {
        for (k=0;k<blkSize;k++)
        {
          pDst[k*dstStride] = Clip3(0, (1<<bitDepth)-1, pDst[k*dstStride] + (( refSide[k+1] - refSide[0] ) >> 1) );
        }
      }
    }
    else
    {
      Int deltaPos=0;
      Int deltaInt;
      Int deltaFract;
      Int refMainIndex;

      for (k=0;k<blkSize;k++)
      {
        deltaPos += intraPredAngle;
        deltaInt   = deltaPos >> 5;
        deltaFract = deltaPos & (32 - 1);

        if (deltaFract)
        {
          // Do linear filtering
          for (l=0;l<blkSize;l++)
          {
            refMainIndex        = l+deltaInt+1;
            pDst[k*dstStride+l] = (Pel) ( ((32-deltaFract)*refMain[refMainIndex]+deltaFract*refMain[refMainIndex+1]+16) >> 5 );
          }
        }
        else
        {
          // Just copy the integer samples
          for (l=0;l<blkSize;l++)
          {
            pDst[k*dstStride+l] = refMain[l+deltaInt+1];
          }
        }
      }
    }

    // Flip the block if this is the horizontal mode
    if (modeHor)
    {
      Pel  tmp;
      for (k=0;k<blkSize-1;k++)
      {
        for (l=k+1;l<blkSize;l++)
        {
          tmp                 = pDst[k*dstStride+l];
          pDst[k*dstStride+l] = pDst[l*dstStride+k];
          pDst[l*dstStride+k] = tmp;
        }
      }
    }
  }
}

Void TComPrediction::predIntraLumaAng(TComPattern* pcTComPattern, UInt uiDirMode, Pel* piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft )
{
  Pel *pDst = piPred;
  Int *ptrSrc;

  assert( g_aucConvertToBit[ iWidth ] >= 0 ); //   4x  4
  assert( g_aucConvertToBit[ iWidth ] <= 5 ); // 128x128
  assert( iWidth == iHeight  );

  ptrSrc = pcTComPattern->getPredictorPtr( uiDirMode, g_aucConvertToBit[ iWidth ] + 2, m_piYuvExt );

  // get starting pixel in block
  Int sw = 2 * iWidth + 1;

  // Create the prediction
  if ( uiDirMode == PLANAR_IDX )
  {
    xPredIntraPlanar( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight );
  }
  else
  {
    if ( (iWidth > 16) || (iHeight > 16) )
    {
      xPredIntraAng(g_bitDepthY, ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, false );
    }
    else
    {
      xPredIntraAng(g_bitDepthY, ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, true );

      if( (uiDirMode == DC_IDX ) && bAbove && bLeft )
      {
        xDCPredFiltering( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight);
      }
    }
  }
}

// Angular chroma
Void TComPrediction::predIntraChromaAng( Int* piSrc, UInt uiDirMode, Pel* piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft )
{
  Pel *pDst = piPred;
  Int *ptrSrc = piSrc;

  // get starting pixel in block
  Int sw = 2 * iWidth + 1;

  if ( uiDirMode == PLANAR_IDX )
  {
    xPredIntraPlanar( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight );
  }
  else
  {
    // Create the prediction
    xPredIntraAng(g_bitDepthC, ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, false );
  }
}

/** Function for checking identical motion.
 * \param TComDataCU* pcCU
 * \param UInt PartAddr
 */
Bool TComPrediction::xCheckIdenticalMotion ( TComDataCU* pcCU, UInt PartAddr )
{
  if( pcCU->getSlice()->isInterB() && !pcCU->getSlice()->getPPS()->getWPBiPred() )
  {
    if( pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr) >= 0 && pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr) >= 0)
    {
      Int RefPOCL0 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_0, pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr))->getPOC();
      Int RefPOCL1 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_1, pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr))->getPOC();
      if(RefPOCL0 == RefPOCL1 && pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(PartAddr) == pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(PartAddr))
      {
        return true;
      }
    }
  }
  return false;
}


Void TComPrediction::motionCompensation ( TComDataCU* pcCU, TComYuv* pcYuvPred,
#if IT_GT
		   Bool bUseGT,
#endif
		   RefPicList eRefPicList, Int iPartIdx )
{
  Int         iWidth;
  Int         iHeight;
  UInt        uiPartAddr;

  if ( iPartIdx >= 0 )
  {
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );
#if IT_GT
    bUseGT = !pcCU->getMergeFlag(uiPartAddr) && pcCU->getGTFlag(uiPartAddr);
    //bUseGT = true;
#endif
    if ( eRefPicList != REF_PIC_LIST_X )
    {
      if( pcCU->getSlice()->getPPS()->getUseWP())
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, true );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred
#if IT_GT
		  ,bUseGT
#endif
		   );
      }
      if ( pcCU->getSlice()->getPPS()->getUseWP() )
      {
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
    }
    else
    {
      if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred
#if IT_GT
		  ,bUseGT
#endif
		  );
      }
      else
      {
        xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred
#if IT_GT
		  ,bUseGT
#endif
		  );
      }
    }
    return;
  }

  for ( iPartIdx = 0; iPartIdx < pcCU->getNumPartitions(); iPartIdx++ )
  {
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );
#if IT_GT
    bUseGT = !pcCU->getMergeFlag(uiPartAddr) && pcCU->getGTFlag(uiPartAddr);
    //bUseGT = true;
#endif
    if ( eRefPicList != REF_PIC_LIST_X )
    {
      if( pcCU->getSlice()->getPPS()->getUseWP())
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred
#if IT_GT
		  ,bUseGT
#endif
		  , true );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred
#if IT_GT
		  ,bUseGT
#endif
		   );
      }
      if ( pcCU->getSlice()->getPPS()->getUseWP() )
      {
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
    }
    else
    {
      if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred
#if IT_GT
		  ,bUseGT
#endif
		   );
      }
      else
      {
        xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred
#if IT_GT
		  ,bUseGT
#endif
		   );
      }
    }
  }
  return;
}

Void TComPrediction::xPredInterUni ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv*& rpcYuvPred
#if IT_GT
		  ,Bool bUseGT
#endif
		  , Bool bi )
{
  Int         iRefIdx     = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );           assert (iRefIdx >= 0);
  TComMv      cMv         = pcCU->getCUMvField( eRefPicList )->getMv( uiPartAddr );

  pcCU->clipMv(cMv);

#if IT_GT
  TComMv      cGT0         = pcCU->getCUGT0Field( eRefPicList )->getMv( uiPartAddr );
  TComMv      cGT1         = pcCU->getCUGT1Field( eRefPicList )->getMv( uiPartAddr );
  TComMv      cGT2         = pcCU->getCUGT2Field( eRefPicList )->getMv( uiPartAddr );
  TComMv      cGT3         = pcCU->getCUGT3Field( eRefPicList )->getMv( uiPartAddr );
  Bool 		  bGTFlag	   = pcCU->getGTFlag(uiPartAddr);
#endif

  xPredInterLumaBlk  ( pcCU, pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, rpcYuvPred, bi
#if IT_GT
		  ,bUseGT && bGTFlag, &cGT0, &cGT1, &cGT2, &cGT3
#endif
		   );
  xPredInterChromaBlk( pcCU, pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, rpcYuvPred, bi
#if IT_GT
		  ,bUseGT && bGTFlag, &cGT0, &cGT1, &cGT2, &cGT3
#endif
		   );
}

Void TComPrediction::xPredInterBi ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv*& rpcYuvPred
#if IT_GT
		  ,Bool bUseGT
#endif
		   )
{
  TComYuv* pcMbYuv;
  Int      iRefIdx[2] = {-1, -1};

  for ( Int iRefList = 0; iRefList < 2; iRefList++ )
  {
    RefPicList eRefPicList = (iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
    iRefIdx[iRefList] = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );

    if ( iRefIdx[iRefList] < 0 )
    {
      continue;
    }

    assert( iRefIdx[iRefList] < pcCU->getSlice()->getNumRefIdx(eRefPicList) );

    pcMbYuv = &m_acYuvPred[iRefList];
    if( pcCU->getCUMvField( REF_PIC_LIST_0 )->getRefIdx( uiPartAddr ) >= 0 && pcCU->getCUMvField( REF_PIC_LIST_1 )->getRefIdx( uiPartAddr ) >= 0 )
    {
      xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv
#if IT_GT
		  ,bUseGT
#endif
		  , true );
    }
    else
    {
      if ( ( pcCU->getSlice()->getPPS()->getUseWP()       && pcCU->getSlice()->getSliceType() == P_SLICE ) || 
           ( pcCU->getSlice()->getPPS()->getWPBiPred() && pcCU->getSlice()->getSliceType() == B_SLICE ) )
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv
#if IT_GT
		  ,bUseGT
#endif
		  , true );
      }
      else
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv
#if IT_GT
		  ,bUseGT
#endif
		   );
      }
    }
  }

  if ( pcCU->getSlice()->getPPS()->getWPBiPred() && pcCU->getSlice()->getSliceType() == B_SLICE  )
  {
    xWeightedPredictionBi( pcCU, &m_acYuvPred[0], &m_acYuvPred[1], iRefIdx[0], iRefIdx[1], uiPartAddr, iWidth, iHeight, rpcYuvPred );
  }  
  else if ( pcCU->getSlice()->getPPS()->getUseWP() && pcCU->getSlice()->getSliceType() == P_SLICE )
  {
    xWeightedPredictionUni( pcCU, &m_acYuvPred[0], uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, rpcYuvPred ); 
  }
  else
  {
    xWeightedAverage( &m_acYuvPred[0], &m_acYuvPred[1], iRefIdx[0], iRefIdx[1], uiPartAddr, iWidth, iHeight, rpcYuvPred );
  }
}

/**
 * \brief Generate motion-compensated luma block
 *
 * \param cu       Pointer to current CU
 * \param refPic   Pointer to reference picture
 * \param partAddr Address of block within CU
 * \param mv       Motion vector
 * \param width    Width of block
 * \param height   Height of block
 * \param dstPic   Pointer to destination picture
 * \param bi       Flag indicating whether bipred is used
 */
Void TComPrediction::xPredInterLumaBlk( TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *&dstPic, Bool bi
#if IT_GT
		, Bool bUseGT
		  ,TComMv *mGT0
		  ,TComMv *mGT1
		  ,TComMv *mGT2
		  ,TComMv *mGT3
#endif
		)
{
#if IT_GT
	if(!bUseGT || (mGT0->getHor() == 0 && mGT1->getHor() == 0 && mGT2->getHor() == 0 && mGT3->getHor() == 0 &&
				mGT0->getVer() == 0 && mGT1->getVer() == 0 && mGT2->getVer() == 0 && mGT3->getVer() == 0)){
#endif

		Int refStride = refPic->getStride();
		Int refOffset = ( mv->getHor() >> 2 ) + ( mv->getVer() >> 2 ) * refStride;
		Pel *ref      = refPic->getLumaAddr( cu->getAddr(), cu->getZorderIdxInCU() + partAddr ) + refOffset;
		Int dstStride = dstPic->getStride();
		Pel *dst      = dstPic->getLumaAddr( partAddr );
		Int xFrac = mv->getHor() & 0x3;
		Int yFrac = mv->getVer() & 0x3;

		if ( yFrac == 0 )
		{
			m_if.filterHorLuma( ref, refStride, dst, dstStride, width, height, xFrac,       !bi );
		}
		else if ( xFrac == 0 )
		{
			m_if.filterVerLuma( ref, refStride, dst, dstStride, width, height, yFrac, true, !bi );
		}
		else
		{
			Int tmpStride = m_filteredBlockTmp[0].getStride();
			Short *tmp    = m_filteredBlockTmp[0].getLumaAddr();
			Int filterSize = NTAPS_LUMA;
			Int halfFilterSize = ( filterSize >> 1 );
			m_if.filterHorLuma(ref - (halfFilterSize-1)*refStride, refStride, tmp, tmpStride, width, height+filterSize-1, xFrac, false     );
			m_if.filterVerLuma(tmp + (halfFilterSize-1)*tmpStride, tmpStride, dst, dstStride, width, height,              yFrac, false, !bi);
		}

#if IT_GT
	}else{
		Int refStride = refPic->getStride();
		Int refOffset = ( mv->getHor() >> 2 ) - width / 2 + (( mv->getVer() >> 2 ) - height / 2) * refStride;
		Pel *ref      = refPic->getLumaAddr( cu->getAddr(), cu->getZorderIdxInCU() + partAddr ) + refOffset;
		Int dstStride = dstPic->getStride();
		Pel *dst      = dstPic->getLumaAddr( partAddr );
		Pel* piDstYorig = dst;
		Pel* piRefYorig = ref;
		Int xFrac = mv->getHor() & 0x3;
		Int yFrac = mv->getVer() & 0x3;
		Pel *dst1Org = (Pel*)xMalloc(Pel, (width * height * 4));
		Pel *dst1 = dst1Org;
		Int dst1Stride = width * 2;

		if ( yFrac == 0 )
		{
			m_if.filterHorLuma( ref, refStride, dst1, dst1Stride, width * 2, height * 2, xFrac,       !bi );
		}
		else if ( xFrac == 0 )
		{
			m_if.filterVerLuma( ref, refStride, dst1, dst1Stride, width * 2, height * 2, yFrac, true, !bi );
		}
		else
		{
			Int tmpStride = m_filteredBlockTmp[0].getStride();
			Short *tmp    = m_filteredBlockTmp[0].getLumaAddr();

			Int filterSize = NTAPS_LUMA;
			Int halfFilterSize = ( filterSize >> 1 );

			m_if.filterHorLuma(ref - (halfFilterSize-1)*refStride, refStride, tmp, tmpStride, width * 2, height * 2 +filterSize-1, xFrac, false     );
			m_if.filterVerLuma(tmp + (halfFilterSize-1)*tmpStride, tmpStride, dst1, dst1Stride, width * 2, height * 2,              yFrac, false, !bi);
		}

		xPredGTLuma(piDstYorig, dst1, height, width, mGT0, mGT1, mGT2, mGT3, dstStride, dst1Stride);

		xFree(dst1Org);
	}
#endif
}

#if IT_GT
Void TComPrediction::xPredGTLuma(Pel* dst, Pel* dst1, Int height, Int width, TComMv* mGT0, TComMv* mGT1, TComMv* mGT2, TComMv* mGT3, Int dstStride, Int dst1Stride)
{
	Int iCurrCornerX[4], iCurrCornerY[4];
	Double dProjective[9];
	Pel* piAuxOrg = (Pel*)xMalloc(Pel, (width * height));
	Pel* piAux = piAuxOrg;;
	Int iNSSWindow = ((height < width) ? (height) : (width)) >> 1;
#if IT_GT_GRID_SIZE > 1
	iNSSWindow *= IT_GT_GRID_SIZE;
#endif
	Int iMaxNSSIteration = IT_MAX_NSS_Iteration;
#if IT_GT_Iteration_Limit
	iMaxNSSIteration = log2((double)(iNSSWindow/IT_GT_GRID_SIZE));
#endif
	/////////////////// TESTING ////////////////
	/*
		 *
	Int lastIterationStep = 1;
	iMaxNSSIteration = log2(double(iNSSWindow));
	if(iMaxNSSIteration <= IT_MAX_NSS_Iteration)
		lastIterationStep = iNSSWindow >> iMaxNSSIteration;
	else
	{
		lastIterationStep = iNSSWindow >> IT_MAX_NSS_Iteration;
		iMaxNSSIteration = IT_MAX_NSS_Iteration;
	}
	*/
	 Int lastIterationStep = iNSSWindow >> iMaxNSSIteration;
	if(lastIterationStep == 0)
		lastIterationStep = 1;
	/////////////////// TESTING ////////////////

#if IT_GT_SEARCH == 0 || IT_GT_SEARCH == 2 || IT_GT_SEARCH == 4
#if IT_GT_GRID_SIZE < 2
	iCurrCornerX[0] = mGT0->getHor() * lastIterationStep;            	iCurrCornerY[0] = mGT0->getVer() * lastIterationStep;
	iCurrCornerX[1] = (mGT1->getHor() * lastIterationStep) + width -1;  iCurrCornerY[1] = mGT1->getVer() * lastIterationStep;
	iCurrCornerX[2] = (mGT2->getHor() * lastIterationStep) + width -1;  iCurrCornerY[2] = (mGT2->getVer() * lastIterationStep) + height -1;
	iCurrCornerX[3] = mGT3->getHor() * lastIterationStep;            	iCurrCornerY[3] = (mGT3->getVer() * lastIterationStep) + height -1;
#else
	iCurrCornerX[0] = mGT0->getHor() * lastIterationStep;            					iCurrCornerY[0] = mGT0->getVer() * lastIterationStep;
	iCurrCornerX[1] = (mGT1->getHor() * lastIterationStep) + width*IT_GT_GRID_SIZE -1;  iCurrCornerY[1] = mGT1->getVer() * lastIterationStep;
	iCurrCornerX[2] = (mGT2->getHor() * lastIterationStep) + width*IT_GT_GRID_SIZE -1;  iCurrCornerY[2] = (mGT2->getVer() * lastIterationStep) + height*IT_GT_GRID_SIZE -1;
	iCurrCornerX[3] = mGT3->getHor() * lastIterationStep;            					iCurrCornerY[3] = (mGT3->getVer() * lastIterationStep) + height*IT_GT_GRID_SIZE -1;
#endif
#else
#if IT_GT_SEARCH == 1
	iCurrCornerX[0] = mGT0->getHor() ;            	 iCurrCornerY[0] = mGT0->getVer() ;
	iCurrCornerX[1] = (mGT1->getHor() ) + width -1;  iCurrCornerY[1] = mGT1->getVer() ;
	iCurrCornerX[2] = (mGT2->getHor() ) + width -1;  iCurrCornerY[2] = (mGT2->getVer() ) + height -1;
	iCurrCornerX[3] = mGT3->getHor() ;            	 iCurrCornerY[3] = (mGT3->getVer() ) + height -1;
#endif
#endif

	dst1 += width / 2 + (height / 2) * dst1Stride;
#if IT_GT_GRID_SIZE < 2
#if !IT_GT_BILINEAR_TRANSFORMATION
	calcParamProjective(iCurrCornerX, iCurrCornerY, dProjective, width , height );
	ProjectiveTransform(dst1, piAux, dProjective, width , height , dst1Stride, (((height < width) ? (height) : (width)) >> 1)); // 64x32 -> 32 -> Window 16x16);
#else
	calcParamBilinear(iCurrCornerX, iCurrCornerY, dProjective, width , height );
	BilinearTransform(dst1, piAux, dProjective, width , height , dst1Stride, (((height < width) ? (height) : (width)) >> 1)); // 64x32 -> 32 -> Window 16x16);
#endif
#else
#if !IT_GT_BILINEAR_TRANSFORMATION
	calcParamProjective(iCurrCornerX, iCurrCornerY, dProjective, width * IT_GT_GRID_SIZE, height * IT_GT_GRID_SIZE);
	ProjectiveTransform(dst1, piAux, dProjective, width * IT_GT_GRID_SIZE, height * IT_GT_GRID_SIZE, dst1Stride, (((height < width) ? (height) : (width)) >> 1)* IT_GT_GRID_SIZE); // 64x32 -> 32 -> Window 16x16);
#else
	calcParamBilinear(iCurrCornerX, iCurrCornerY, dProjective, width * IT_GT_GRID_SIZE, height * IT_GT_GRID_SIZE);
	BilinearTransform(dst1, piAux, dProjective, width * IT_GT_GRID_SIZE, height * IT_GT_GRID_SIZE, dst1Stride, (((height < width) ? (height) : (width)) >> 1)* IT_GT_GRID_SIZE); // 64x32 -> 32 -> Window 16x16);
#endif
#endif

	for(Int y = 0; y < height; y++)
	{
		for(Int x = 0; x < width; x++)
		{
			dst[x] = piAux[x];
		}
		dst += dstStride;
		piAux += width;
	}
	xFree(piAuxOrg);
}

Void TComPrediction::calcParamProjective(Int x[4], Int y[4], Double h[9], Int Width, Int Height)
{
  Double H, W, deltax[4], deltay[4];

  W = (Double)Width - 1.0;
  H = (Double)Height - 1.0;

  deltax[1] = (Double)x[1] - x[2];
  deltax[2] = (Double)x[3] - x[2];
  deltax[3] = (Double)x[0] - x[1] + x[2] - x[3];
  deltay[1] = (Double)y[1] - y[2];
  deltay[2] = (Double)y[3] - y[2];
  deltay[3] = (Double)y[0] - y[1] + y[2] - y[3];

  h[2] = ((deltax[3] * deltay[2] - deltax[2] * deltay[3]) / (deltax[1] * deltay[2] - deltax[2] * deltay[1])) / W;
  h[5] = ((deltax[1] * deltay[3] - deltax[3] * deltay[1]) / (deltax[1] * deltay[2] - deltax[2] * deltay[1])) / H;

  h[0] = (Double)(x[1] - x[0]) / W + h[2] * x[1];
  h[3] = (Double)(x[3] - x[0]) / H + h[5] * x[3];
  h[6] = (Double)x[0];
  h[1] = (Double)(y[1] - y[0]) / W + h[2] * y[1];
  h[4] = (Double)(y[3] - y[0]) / H + h[5] * y[3];
  h[7] = (Double)y[0];

  h[8] = 1.0;
}
#if IT_GT_UV
Void TComPrediction::calcParamProjectiveC(Double x[4], Double y[4], Double h[9], Int Width, Int Height)
{
  Double H, W, deltax[4], deltay[4];

  W = (Double)Width - 1.0;
  H = (Double)Height - 1.0;

  deltax[1] = x[1] - x[2];
  deltax[2] = x[3] - x[2];
  deltax[3] = x[0] - x[1] + x[2] - x[3];
  deltay[1] = y[1] - y[2];
  deltay[2] = y[3] - y[2];
  deltay[3] = y[0] - y[1] + y[2] - y[3];

  h[2] = ((deltax[3] * deltay[2] - deltax[2] * deltay[3]) / (deltax[1] * deltay[2] - deltax[2] * deltay[1])) / W;
  h[5] = ((deltax[1] * deltay[3] - deltax[3] * deltay[1]) / (deltax[1] * deltay[2] - deltax[2] * deltay[1])) / H;

  h[0] = (x[1] - x[0]) / W + h[2] * x[1];
  h[3] = (x[3] - x[0]) / H + h[5] * x[3];
  h[6] = x[0];
  h[1] = (y[1] - y[0]) / W + h[2] * y[1];
  h[4] = (y[3] - y[0]) / H + h[5] * y[3];
  h[7] = y[0];

  h[8] = 1.0;
}
#endif

Void TComPrediction::calcParamBilinear(Int x[4], Int y[4], Double h[9], Int Width, Int Height)
{
  Double H, W;

  W = (Double)Width - 1.0;
  H = (Double)Height - 1.0;

  h[0] = x[0];
  h[1] = (x[1]-x[0])/(H);
  h[2] = (x[3]-x[0])/(W);
  h[3] = (x[2]-x[3]-x[1]+x[0])/(H*W);

  h[4] = y[0];
  h[5] = (y[1]-y[0])/(H);
  h[6] = (y[3]-y[0])/(W);
  h[7] = (y[2]-y[3]-y[1]+y[0])/(H*W);

  h[8] = 0.0; // unused

}
#if IT_GT_UV
Void TComPrediction::calcParamBilinearC(Double x[4], Double y[4], Double h[9], Int Width, Int Height)
{
  Double H, W;

  W = (Double)Width - 1.0;
  H = (Double)Height - 1.0;

  h[0] = x[0];
  h[1] = (x[1]-x[0])/(H);
  h[2] = (x[3]-x[0])/(W);
  h[3] = (x[2]-x[3]-x[1]+x[0])/(H*W);

  h[4] = y[0];
  h[5] = (y[1]-y[0])/(H);
  h[6] = (y[3]-y[0])/(W);
  h[7] = (y[2]-y[3]-y[1]+y[0])/(H*W);

  h[8] = 0.0; // unused

}
#endif
Void TComPrediction::ProjectiveTransform(Pel* piRefY, Pel* piAux, Double h[9], Int W, Int H, Int iStrideCur, Int iNSSWindow)
{
  Int X = 0, Y = 0, pixels = 0;
  Double Fx = 0.0, Fy = 0.0, q = 0.0, p = 0.0, aux = 0.0;
  Pel *paux;

#if IT_GT_Interpolation_Filter == 2
  Double DCTIF[IT_GT_Interpolation_Filter_order];
  Double pixel[IT_GT_Interpolation_Filter_order];
#endif

#if IT_GT_GRID_SIZE < 2
  for (Int y = 0; y < H; y++){
    for (Int x = 0; x < W; x++){
#else
    	Int offsetX = W/2 - (W / IT_GT_GRID_SIZE / 2);
    	Int offsetY = H/2 - (H / IT_GT_GRID_SIZE / 2);
    	for (Int y = offsetY; y < offsetY + H / IT_GT_GRID_SIZE ; y++){
    	    for (Int x = offsetX; x < offsetX + W / IT_GT_GRID_SIZE; x++){
#endif
      // New coordinates after the deformation
      Fx = (h[0] * x + h[3] * y + h[6]) / (h[2] * x + h[5] * y + h[8]);
      Fy = (h[1] * x + h[4] * y + h[7]) / (h[2] * x + h[5] * y + h[8]);
      // Difference to integer
#if IT_GT_GRID_SIZE < 2
      Y = (Int)Fy;
      X = (Int)Fx;
      q = (Fy - (Double)Y);
      p = (Fx - (Double)X);
      if (Y < -iNSSWindow)
        Y = -iNSSWindow;
      if (X < -iNSSWindow)
        X = -iNSSWindow;
      if (Y > iNSSWindow + H - 1)
        Y = iNSSWindow + H - 1;
      if (X > iNSSWindow + W - 1)
        X = iNSSWindow + W - 1;
      if (Y + 1 > iNSSWindow + H - 1)
        Y = iNSSWindow + H - 2;
      if (X + 1 > iNSSWindow + W - 1)
        X = iNSSWindow + W - 2;
#else
      Y = (Int)Fy - offsetY;
      X = (Int)Fx - offsetX;
      q = (Fy - offsetY - (Double)Y);
      p = (Fx - offsetX - (Double)X);
      if (Y < -iNSSWindow/IT_GT_GRID_SIZE)
    	  Y = -iNSSWindow/IT_GT_GRID_SIZE;
      if (X < -iNSSWindow/IT_GT_GRID_SIZE)
    	  X = -iNSSWindow/IT_GT_GRID_SIZE;
      if (Y > iNSSWindow/IT_GT_GRID_SIZE + H/IT_GT_GRID_SIZE - 1)
    	  Y = iNSSWindow/IT_GT_GRID_SIZE + H/IT_GT_GRID_SIZE - 1;
      if (X > iNSSWindow/IT_GT_GRID_SIZE + W/IT_GT_GRID_SIZE - 1)
    	  X = iNSSWindow/IT_GT_GRID_SIZE + W/IT_GT_GRID_SIZE - 1;
      if (Y + 1 > iNSSWindow/IT_GT_GRID_SIZE + H/IT_GT_GRID_SIZE - 1)
    	  Y = iNSSWindow/IT_GT_GRID_SIZE + H/IT_GT_GRID_SIZE - 2;
      if (X + 1 > iNSSWindow/IT_GT_GRID_SIZE + W/IT_GT_GRID_SIZE - 1)
    	  X = iNSSWindow/IT_GT_GRID_SIZE + W/IT_GT_GRID_SIZE - 2;
#endif
      paux = piRefY + Y * iStrideCur;
      // interpolation filter
#if IT_GT_Interpolation_Filter == 0
      aux = (1.0 - q) * ((1.0 - p)* (Double)(paux[X]) + p* (Double)(paux[X + 1]));
      paux = piRefY + (Y + 1)* iStrideCur;
      aux += q  * ((1.0 - p)* (Double)(paux[X]) + p* (Double)(paux[X + 1]));
      if (aux > 255)
        aux = 255;
      if (aux < 0)
        aux = 0;
#endif
#if IT_GT_Interpolation_Filter == 1
      if(p < 0.5 && q < 0.5) // up left
    	  aux = paux[X];
      else if(p >= 0.5 && q < 0.5) // up right
      {
    	  aux = paux[X+1];
      }
      else if(p < 0.5 && q >= 0.5) // down left
      {
    	  paux = piRefY + (Y + 1)* iStrideCur;
    	  aux = paux[X];
      }
      else if(p >= 0.5 && q >= 0.5) // down right
      {
    	  paux = piRefY + (Y + 1)* iStrideCur;
    	  aux = paux[X+1];
      }
#endif
#if IT_GT_Interpolation_Filter == 2
      if(q == 0 && p != 0) // horizontal
      {
          designFilter(DCTIF, p);
          aux = applyFilterHor(piRefY, iStrideCur, X, Y, DCTIF, W, H, iNSSWindow);
      }
      else if(p == 0 && q != 0) // vertical
      {
          designFilter(DCTIF, q);
          aux = applyFilterVer(piRefY, iStrideCur, X, Y, DCTIF, W, H, iNSSWindow);
      }
      else // both
      {
    	  Int M = IT_GT_Interpolation_Filter_order / 2;
    	  Int y = 0;
          designFilter(DCTIF, p);
      	  for(Int m = 1 - M; m <= M; m++)
      	  { // several horizontal filters
      		  y = Y + m;
      		  if(y < -iNSSWindow/IT_GT_GRID_SIZE)
      			  y = -iNSSWindow/IT_GT_GRID_SIZE;
      		  if(y > iNSSWindow/IT_GT_GRID_SIZE + H/IT_GT_GRID_SIZE - 1)
      			  y = iNSSWindow/IT_GT_GRID_SIZE + H/IT_GT_GRID_SIZE - 1;
      		  pixel[m - (1 - M)] = applyFilterHor(piRefY, iStrideCur, X, y, DCTIF, W, H, iNSSWindow);
      	  }
      	  designFilter(DCTIF, q);
      	  aux = applyFilterHor(pixel, DCTIF);
      }

#endif
#if IT_GT_GRID_SIZE < 2
      piAux[x] = (Pel)(aux + 0.5);
#else
      piAux[x-offsetX] = (Pel)(aux + 0.5);
#endif
    }
    piAux += W/IT_GT_GRID_SIZE;
  }
}

#if IT_GT_Interpolation_Filter == 2
Void TComPrediction::designFilter (Double DCTIF[IT_GT_Interpolation_Filter_order], Double alpha)
{
	Double nTaps = IT_GT_Interpolation_Filter_order;
	Double M = nTaps / 2;
	Double N = nTaps + 3;
	Double c[IT_GT_Interpolation_Filter_order];
	Double total = 0;

	for(Int k = 0; k < nTaps; k++)
	{
		c[k] = 1;
		DCTIF[k] = 0;
	}
	c[0] = (1/sqrt(2)) * (1/sqrt(2));

	// DCTIF
    for(Int m = 1 - M; m <= M; m++)
    {
        for(Int k = 0; k < nTaps; k++)
        {
        	DCTIF[m-(1-Int(M))] += c[k] * cos(((2 * m - 1 + 2 * M) * PI * k) / (4 * M)) * cos(((2 * alpha - 1 + 2 * M) * PI * k) / (4 * M));
        	//cout << c[k] << " x " << cos(((2 * alpha - 1 + 2 * M) * PI * k) / (4 * M)) << " x " << cos(((2 * m - 1 + 2 * M) * PI * k) / (4 * M)) << endl;
        }
        DCTIF[m-(1-Int(M))] *= (1 / M);
    }
    // smoothing window
    for(Int m = 1 - M; m <= M; m++)
    	DCTIF[m-(1-Int(M))] *= cos(PI*((m - alpha)/(N - 1)));

    // normalize
    for(Int m = 1 - M; m <= M; m++)
    	total += DCTIF[m-(1-Int(M))];
    for(Int m = 1 - M; m <= M; m++)
    	DCTIF[m-(1-Int(M))] /= total;

}

Double TComPrediction::applyFilterHor(Pel *piRefY, Int iStrideCur, Int X, Int Y, Double DCTIF[IT_GT_Interpolation_Filter_order], Int W, Int H, Int iNSSWindow)
{
	Int nTaps = IT_GT_Interpolation_Filter_order;
	Int M = nTaps >> 1;
	Int N = nTaps + 3;
	Int xMinLimit = 0;
	Int xMaxLimit = 0;
	Int gridSize = IT_GT_GRID_SIZE;
	Int x = 0;
	Double pixel = 0;
	Pel *aux = piRefY + Y * iStrideCur;

	xMinLimit = -iNSSWindow/gridSize;
	xMaxLimit = iNSSWindow/gridSize + W/gridSize - 1;

	for(Int m = 1 - M; m <= M; m++)
	{
		x = X + m;
		if(x < xMinLimit)
			x = xMinLimit;
		if(x > xMaxLimit)
			x = xMaxLimit;

		pixel += Double(aux[x]) * DCTIF[m - (1 - M)];
		//cout << Double(aux[x]) << " x " << DCTIF[m - (1 - M)] << "  ";
	}
	//cout << pixel << endl;
	return pixel;
}

Double TComPrediction::applyFilterVer(Pel *piRefY, Int iStrideCur, Int X, Int Y, Double DCTIF[IT_GT_Interpolation_Filter_order], Int W, Int H, Int iNSSWindow)
{
	Int nTaps = IT_GT_Interpolation_Filter_order;
	Int M = nTaps >> 1;
	Int N = nTaps + 3;
	Int yMinLimit = 0;
	Int yMaxLimit = 0;
	Int gridSize = IT_GT_GRID_SIZE;
	Int y = 0;
	Double pixel = 0;
	Pel *aux = piRefY + Y * iStrideCur;

	yMinLimit = -iNSSWindow/gridSize;
	yMaxLimit = iNSSWindow/gridSize + H/gridSize - 1;

	for(Int m = 1 - M; m <= M; m++)
	{
		y = Y + m;
		if(y < yMinLimit)
			y = yMinLimit;
		if(y > yMaxLimit)
			y = yMaxLimit;
		aux = piRefY + y * iStrideCur;
		pixel += Double(aux[X]) * DCTIF[m - (1 - M)];
	}
	return pixel;
}

Double TComPrediction::applyFilterHor(Double *piRefY, Double DCTIF[IT_GT_Interpolation_Filter_order])
{
	Int nTaps = IT_GT_Interpolation_Filter_order;
	Int M = nTaps >> 1;
	Int N = nTaps + 3;
	Int xMinLimit = 0;
	Int xMaxLimit = 0;
	Int gridSize = IT_GT_GRID_SIZE;
	Int x = 0;
	Double pixel = 0;

	for(Int m = 1 - M; m <= M; m++)
	{
		pixel += Double(piRefY[m - (1 - M)]) * DCTIF[m - (1 - M)];
	}
	return pixel;
}

#endif

Void TComPrediction::BilinearTransform(Pel* piRefY, Pel* piAux, Double h[9], Int W, Int H, Int iStrideCur, Int iNSSWindow)
{
  Int X = 0, Y = 0, pixels = 0;
  Double Fx = 0.0, Fy = 0.0, q = 0.0, p = 0.0, aux = 0.0;
  Pel *paux;


#if IT_GT_GRID_SIZE < 2
  for (Int y = 0; y < H; y++){
    for (Int x = 0; x < W; x++){
#else
    	Int offsetX = W/2 - (W / IT_GT_GRID_SIZE / 2);
    	Int offsetY = H/2 - (H / IT_GT_GRID_SIZE / 2);
    	for (Int y = offsetY; y < offsetY + H / IT_GT_GRID_SIZE ; y++){
    	    for (Int x = offsetX; x < offsetX + W / IT_GT_GRID_SIZE; x++){
#endif
      // New coordinates after the deformation
      Fx = h[0] + h[1]*x + h[2]*y + h[3]*x*y;
      Fy = h[4] + h[5]*x + h[6]*y + h[7]*x*y;
      // Difference to integer
#if IT_GT_GRID_SIZE < 2
      Y = (Int)Fy;
      X = (Int)Fx;
      q = (Fy - (Double)Y);
      p = (Fx - (Double)X);
      if (Y < -iNSSWindow)
        Y = -iNSSWindow;
      if (X < -iNSSWindow)
        X = -iNSSWindow;
      if (Y > iNSSWindow + H - 1)
        Y = iNSSWindow + H - 1;
      if (X > iNSSWindow + W - 1)
        X = iNSSWindow + W - 1;
      if (Y + 1 > iNSSWindow + H - 1)
        Y = iNSSWindow + H - 2;
      if (X + 1 > iNSSWindow + W - 1)
        X = iNSSWindow + W - 2;
#else
      Y = (Int)Fy - offsetY;
      X = (Int)Fx - offsetX;
      q = (Fy - offsetY - (Double)Y);
      p = (Fx - offsetX - (Double)X);
      if (Y < -iNSSWindow/IT_GT_GRID_SIZE)
    	  Y = -iNSSWindow/IT_GT_GRID_SIZE;
      if (X < -iNSSWindow/IT_GT_GRID_SIZE)
    	  X = -iNSSWindow/IT_GT_GRID_SIZE;
      if (Y > iNSSWindow/IT_GT_GRID_SIZE + H/IT_GT_GRID_SIZE - 1)
    	  Y = iNSSWindow/IT_GT_GRID_SIZE + H/IT_GT_GRID_SIZE - 1;
      if (X > iNSSWindow/IT_GT_GRID_SIZE + W/IT_GT_GRID_SIZE - 1)
    	  X = iNSSWindow/IT_GT_GRID_SIZE + W/IT_GT_GRID_SIZE - 1;
      if (Y + 1 > iNSSWindow/IT_GT_GRID_SIZE + H/IT_GT_GRID_SIZE - 1)
    	  Y = iNSSWindow/IT_GT_GRID_SIZE + H/IT_GT_GRID_SIZE - 2;
      if (X + 1 > iNSSWindow/IT_GT_GRID_SIZE + W/IT_GT_GRID_SIZE - 1)
    	  X = iNSSWindow/IT_GT_GRID_SIZE + W/IT_GT_GRID_SIZE - 2;
#endif
      // bilinear interpolation
      paux = piRefY + Y * iStrideCur;
      aux = (1.0 - q) * ((1.0 - p)* (Double)(paux[X]) + p* (Double)(paux[X + 1]));
      paux = piRefY + (Y + 1)* iStrideCur;
      aux += q  * ((1.0 - p)* (Double)(paux[X]) + p* (Double)(paux[X + 1]));
      if (aux + 0.5 > 255)
        aux = 255;
      if (aux + 0.5 < 0)
        aux = 0;
#if IT_GT_GRID_SIZE < 2
      piAux[x] = (Pel)(aux + 0.5);
#else
      piAux[x-offsetX] = (Pel)(aux + 0.5);
#endif
    }
    piAux += W/IT_GT_GRID_SIZE;
  }
}
#endif

/**
 * \brief Generate motion-compensated chroma block
 *
 * \param cu       Pointer to current CU
 * \param refPic   Pointer to reference picture
 * \param partAddr Address of block within CU
 * \param mv       Motion vector
 * \param width    Width of block
 * \param height   Height of block
 * \param dstPic   Pointer to destination picture
 * \param bi       Flag indicating whether bipred is used
 */
Void TComPrediction::xPredInterChromaBlk( TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *&dstPic, Bool bi
#if IT_GT
		, Bool bUseGT
		  ,TComMv *mGT0
		  ,TComMv *mGT1
		  ,TComMv *mGT2
		  ,TComMv *mGT3
#endif
		)
{
#if IT_GT && IT_GT_UV
	if(!bUseGT || (mGT0->getHor() == 0 && mGT1->getHor() == 0 && mGT2->getHor() == 0 && mGT3->getHor() == 0 &&
				mGT0->getVer() == 0 && mGT1->getVer() == 0 && mGT2->getVer() == 0 && mGT3->getVer() == 0)){
#endif
  Int     refStride  = refPic->getCStride();
  Int     dstStride  = dstPic->getCStride();
  
  Int     refOffset  = (mv->getHor() >> 3) + (mv->getVer() >> 3) * refStride;
  
  Pel*    refCb     = refPic->getCbAddr( cu->getAddr(), cu->getZorderIdxInCU() + partAddr ) + refOffset;
  Pel*    refCr     = refPic->getCrAddr( cu->getAddr(), cu->getZorderIdxInCU() + partAddr ) + refOffset;
  
  Pel* dstCb = dstPic->getCbAddr( partAddr );
  Pel* dstCr = dstPic->getCrAddr( partAddr );
  
  Int     xFrac  = mv->getHor() & 0x7;
  Int     yFrac  = mv->getVer() & 0x7;
  UInt    cxWidth  = width  >> 1;
  UInt    cxHeight = height >> 1;
  
  Int     extStride = m_filteredBlockTmp[0].getStride();
  Short*  extY      = m_filteredBlockTmp[0].getLumaAddr();
  
  Int filterSize = NTAPS_CHROMA;
  
  Int halfFilterSize = (filterSize>>1);
  
  if ( yFrac == 0 )
  {
    m_if.filterHorChroma(refCb, refStride, dstCb,  dstStride, cxWidth, cxHeight, xFrac, !bi);    
    m_if.filterHorChroma(refCr, refStride, dstCr,  dstStride, cxWidth, cxHeight, xFrac, !bi);    
  }
  else if ( xFrac == 0 )
  {
    m_if.filterVerChroma(refCb, refStride, dstCb, dstStride, cxWidth, cxHeight, yFrac, true, !bi);    
    m_if.filterVerChroma(refCr, refStride, dstCr, dstStride, cxWidth, cxHeight, yFrac, true, !bi);    
  }
  else
  {
    m_if.filterHorChroma(refCb - (halfFilterSize-1)*refStride, refStride, extY,  extStride, cxWidth, cxHeight+filterSize-1, xFrac, false);
    m_if.filterVerChroma(extY  + (halfFilterSize-1)*extStride, extStride, dstCb, dstStride, cxWidth, cxHeight  , yFrac, false, !bi);
    
    m_if.filterHorChroma(refCr - (halfFilterSize-1)*refStride, refStride, extY,  extStride, cxWidth, cxHeight+filterSize-1, xFrac, false);
    m_if.filterVerChroma(extY  + (halfFilterSize-1)*extStride, extStride, dstCr, dstStride, cxWidth, cxHeight  , yFrac, false, !bi);    
  }
#if IT_GT && IT_GT_UV
	}else{

		Int     refStride  = refPic->getCStride();
		Int     dstStride  = dstPic->getCStride();
		Int 	refOffset = ( mv->getHor() >> 3 ) - width / 4 + (( mv->getVer() >> 3 ) - height / 4) * refStride;
		Pel*    refCb     = refPic->getCbAddr( cu->getAddr(), cu->getZorderIdxInCU() + partAddr ) + refOffset;
		Pel*    refCr     = refPic->getCrAddr( cu->getAddr(), cu->getZorderIdxInCU() + partAddr ) + refOffset;
		Pel* 	dstCb = dstPic->getCbAddr( partAddr );
		Pel* 	dstCr = dstPic->getCrAddr( partAddr );
		Int     xFrac  = mv->getHor() & 0x7;
		Int     yFrac  = mv->getVer() & 0x7;
		UInt    cxWidth  = width  >> 1;
		UInt    cxHeight = height >> 1;
		Int     extStride = m_filteredBlockTmp[0].getStride();
		Short*  extY      = m_filteredBlockTmp[0].getLumaAddr();
		Int filterSize = NTAPS_CHROMA;
		Int halfFilterSize = (filterSize>>1);

		Pel* piDstCborig = dstCb;
		Pel* piRefCborig = refCb;
		Pel *dst1CbOrg = (Pel*)xMalloc(Pel, (width * height));
		Pel *dst1Cb = dst1CbOrg;

		Pel* piDstCrorig = dstCr;
		Pel* piRefCrorig = refCr;
		Pel *dst1CrOrg = (Pel*)xMalloc(Pel, (width * height));
		Pel *dst1Cr = dst1CrOrg;

		Int dst1Stride = width;


		if ( yFrac == 0 )
		{
			m_if.filterHorChroma(refCb, refStride, dst1Cb,  dst1Stride, cxWidth*2, cxHeight*2, xFrac, !bi);
			m_if.filterHorChroma(refCr, refStride, dst1Cr,  dst1Stride, cxWidth*2, cxHeight*2, xFrac, !bi);
		}
		else if ( xFrac == 0 )
		{
			m_if.filterVerChroma(refCb, refStride, dst1Cb, dst1Stride, cxWidth*2, cxHeight*2, yFrac, true, !bi);
			m_if.filterVerChroma(refCr, refStride, dst1Cr, dst1Stride, cxWidth*2, cxHeight*2, yFrac, true, !bi);
		}
		else
		{
			m_if.filterHorChroma(refCb - (halfFilterSize-1)*refStride, refStride, extY,  extStride, cxWidth*2, cxHeight*2+filterSize-1, xFrac, false);
			m_if.filterVerChroma(extY  + (halfFilterSize-1)*extStride, extStride, dst1Cb, dst1Stride, cxWidth*2, cxHeight*2  , yFrac, false, !bi);

			m_if.filterHorChroma(refCr - (halfFilterSize-1)*refStride, refStride, extY,  extStride, cxWidth*2, cxHeight*2+filterSize-1, xFrac, false);
			m_if.filterVerChroma(extY  + (halfFilterSize-1)*extStride, extStride, dst1Cr, dst1Stride, cxWidth*2, cxHeight*2  , yFrac, false, !bi);
		}

		xPredGTChroma(piDstCborig, dst1Cb, height/2, width/2, mGT0, mGT1, mGT2, mGT3, dstStride, dst1Stride);
		xPredGTChroma(piDstCrorig, dst1Cr, height/2, width/2, mGT0, mGT1, mGT2, mGT3, dstStride, dst1Stride);
		xFree(dst1CrOrg);
		xFree(dst1CbOrg);
  }
#endif
}

#if IT_GT
#if IT_GT_UV
Void TComPrediction::xPredGTChroma(Pel* dst, Pel* dst1, Int height, Int width, TComMv* mGT0, TComMv* mGT1, TComMv* mGT2, TComMv* mGT3, Int dstStride, Int dst1Stride)
{
	Double iCurrCornerX[4], iCurrCornerY[4];
	Double dProjective[9];
	Pel* piAuxOrg = (Pel*)xMalloc(Pel, (width * height));
	Pel* piAux = piAuxOrg;;
	Int iNSSWindow = ((height < width) ? (height) : (width)) >> 1;
#if IT_GT_GRID_SIZE > 1
	iNSSWindow *= IT_GT_GRID_SIZE;
#endif
	Int iMaxNSSIteration = IT_MAX_NSS_Iteration;
#if IT_GT_Iteration_Limit
	iMaxNSSIteration = log2((double)(iNSSWindow/IT_GT_GRID_SIZE));
#endif

	 Double lastIterationStep = iNSSWindow >> iMaxNSSIteration;
	if(lastIterationStep == 0)
		lastIterationStep = 1;

#if IT_GT_SEARCH == 0 || IT_GT_SEARCH == 2 || IT_GT_SEARCH == 4
#if IT_GT_GRID_SIZE < 2
	iCurrCornerX[0] = ((Double)(mGT0->getHor())/2) * lastIterationStep;            	iCurrCornerY[0] = ((Double)(mGT0->getVer())/2) * lastIterationStep;
	iCurrCornerX[1] = (((Double)(mGT1->getHor())/2) * lastIterationStep) + width -1;  iCurrCornerY[1] = ((Double)(mGT1->getVer())/2) * lastIterationStep;
	iCurrCornerX[2] = (((Double)(mGT2->getHor())/2) * lastIterationStep) + width -1;  iCurrCornerY[2] = (((Double)(mGT2->getVer())/2) * lastIterationStep) + height -1;
	iCurrCornerX[3] = ((Double)(mGT3->getHor())/2) * lastIterationStep;            	iCurrCornerY[3] = (((Double)(mGT3->getVer())/2) * lastIterationStep) + height -1;
#else
	iCurrCornerX[0] = ((Double)(mGT0->getHor())/2) * lastIterationStep;            					iCurrCornerY[0] = ((Double)(mGT0->getVer())/2) * lastIterationStep;
	iCurrCornerX[1] = (((Double)(mGT1->getHor())/2) * lastIterationStep) + width*IT_GT_GRID_SIZE -1;  iCurrCornerY[1] = ((Double)(mGT1->getVer())/2) * lastIterationStep;
	iCurrCornerX[2] = (((Double)(mGT2->getHor())/2) * lastIterationStep) + width*IT_GT_GRID_SIZE -1;  iCurrCornerY[2] = (((Double)(mGT2->getVer())/2) * lastIterationStep) + height*IT_GT_GRID_SIZE -1;
	iCurrCornerX[3] = ((Double)(mGT3->getHor())/2) * lastIterationStep;            					iCurrCornerY[3] = (((Double)(mGT3->getVer())/2) * lastIterationStep) + height*IT_GT_GRID_SIZE -1;
#endif
#else
#if IT_GT_SEARCH == 1
	iCurrCornerX[0] = ((Double)(mGT0->getHor())/2) ;            	 iCurrCornerY[0] = ((Double)(mGT0->getVer())/2) ;
	iCurrCornerX[1] = (((Double)(mGT1->getHor())/2) ) + width -1;  iCurrCornerY[1] = ((Double)(mGT1->getVer())/2) ;
	iCurrCornerX[2] = (((Double)(mGT2->getHor())/2) ) + width -1;  iCurrCornerY[2] = (((Double)(mGT2->getVer())/2)) + height -1;
	iCurrCornerX[3] = ((Double)(mGT3->getHor())/2) ;            	 iCurrCornerY[3] = (((Double)(mGT3->getVer())/2)) + height -1;
#endif
#endif

	dst1 += width / 2 + (height / 2) * dst1Stride;
#if IT_GT_GRID_SIZE < 2
#if !IT_GT_BILINEAR_TRANSFORMATION
	calcParamProjectiveC(iCurrCornerX, iCurrCornerY, dProjective, width , height );
	ProjectiveTransform(dst1, piAux, dProjective, width , height , dst1Stride, (((height < width) ? (height) : (width)) >> 1)); // 64x32 -> 32 -> Window 16x16);
#else
	calcParamBilinearC(iCurrCornerX, iCurrCornerY, dProjective, width , height );
	BilinearTransform(dst1, piAux, dProjective, width , height , dst1Stride, (((height < width) ? (height) : (width)) >> 1)); // 64x32 -> 32 -> Window 16x16);
#endif
#else
#if !IT_GT_BILINEAR_TRANSFORMATION
	calcParamProjectiveC(iCurrCornerX, iCurrCornerY, dProjective, width * IT_GT_GRID_SIZE, height * IT_GT_GRID_SIZE);
	ProjectiveTransform(dst1, piAux, dProjective, width * IT_GT_GRID_SIZE, height * IT_GT_GRID_SIZE, dst1Stride, (((height < width) ? (height) : (width)) >> 1)* IT_GT_GRID_SIZE); // 64x32 -> 32 -> Window 16x16);
#else
	calcParamBilinearC(iCurrCornerX, iCurrCornerY, dProjective, width * IT_GT_GRID_SIZE, height * IT_GT_GRID_SIZE);
	BilinearTransform(dst1, piAux, dProjective, width * IT_GT_GRID_SIZE, height * IT_GT_GRID_SIZE, dst1Stride, (((height < width) ? (height) : (width)) >> 1)* IT_GT_GRID_SIZE); // 64x32 -> 32 -> Window 16x16);
#endif
#endif

	for(Int y = 0; y < height; y++)
	{
		for(Int x = 0; x < width; x++)
		{
			dst[x] = piAux[x];
		}
		dst += dstStride;
		piAux += width;
	}
	xFree(piAuxOrg);
}
#endif
#endif

Void TComPrediction::xWeightedAverage( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, Int iRefIdx0, Int iRefIdx1, UInt uiPartIdx, Int iWidth, Int iHeight, TComYuv*& rpcYuvDst )
{
  if( iRefIdx0 >= 0 && iRefIdx1 >= 0 )
  {
    rpcYuvDst->addAvg( pcYuvSrc0, pcYuvSrc1, uiPartIdx, iWidth, iHeight );
  }
  else if ( iRefIdx0 >= 0 && iRefIdx1 <  0 )
  {
    pcYuvSrc0->copyPartToPartYuv( rpcYuvDst, uiPartIdx, iWidth, iHeight );
  }
  else if ( iRefIdx0 <  0 && iRefIdx1 >= 0 )
  {
    pcYuvSrc1->copyPartToPartYuv( rpcYuvDst, uiPartIdx, iWidth, iHeight );
  }
}

// AMVP
Void TComPrediction::getMvPredAMVP( TComDataCU* pcCU, UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, TComMv& rcMvPred )
{
  AMVPInfo* pcAMVPInfo = pcCU->getCUMvField(eRefPicList)->getAMVPInfo();
  if( pcAMVPInfo->iN <= 1 )
  {
    rcMvPred = pcAMVPInfo->m_acMvCand[0];

    pcCU->setMVPIdxSubParts( 0, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( pcAMVPInfo->iN, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    return;
  }

  assert(pcCU->getMVPIdx(eRefPicList,uiPartAddr) >= 0);
  rcMvPred = pcAMVPInfo->m_acMvCand[pcCU->getMVPIdx(eRefPicList,uiPartAddr)];
  return;
}

/** Function for deriving planar intra prediction.
 * \param pSrc pointer to reconstructed sample array
 * \param srcStride the stride of the reconstructed sample array
 * \param rpDst reference to pointer for the prediction sample array
 * \param dstStride the stride of the prediction sample array
 * \param width the width of the block
 * \param height the height of the block
 *
 * This function derives the prediction samples for planar mode (intra coding).
 */
Void TComPrediction::xPredIntraPlanar( Int* pSrc, Int srcStride, Pel* rpDst, Int dstStride, UInt width, UInt height )
{
  assert(width == height);

  Int k, l, bottomLeft, topRight;
  Int horPred;
  Int leftColumn[MAX_CU_SIZE+1], topRow[MAX_CU_SIZE+1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
  UInt blkSize = width;
  UInt offset2D = width;
  UInt shift1D = g_aucConvertToBit[ width ] + 2;
  UInt shift2D = shift1D + 1;

  // Get left and above reference column and row
  for(k=0;k<blkSize+1;k++)
  {
    topRow[k] = pSrc[k-srcStride];
    leftColumn[k] = pSrc[k*srcStride-1];
  }

  // Prepare intermediate variables used in interpolation
  bottomLeft = leftColumn[blkSize];
  topRight   = topRow[blkSize];
  for (k=0;k<blkSize;k++)
  {
    bottomRow[k]   = bottomLeft - topRow[k];
    rightColumn[k] = topRight   - leftColumn[k];
    topRow[k]      <<= shift1D;
    leftColumn[k]  <<= shift1D;
  }

  // Generate prediction signal
  for (k=0;k<blkSize;k++)
  {
    horPred = leftColumn[k] + offset2D;
    for (l=0;l<blkSize;l++)
    {
      horPred += rightColumn[k];
      topRow[l] += bottomRow[l];
      rpDst[k*dstStride+l] = ( (horPred + topRow[l]) >> shift2D );
    }
  }
}

/** Function for filtering intra DC predictor.
 * \param pSrc pointer to reconstructed sample array
 * \param iSrcStride the stride of the reconstructed sample array
 * \param rpDst reference to pointer for the prediction sample array
 * \param iDstStride the stride of the prediction sample array
 * \param iWidth the width of the block
 * \param iHeight the height of the block
 *
 * This function performs filtering left and top edges of the prediction samples for DC mode (intra coding).
 */
Void TComPrediction::xDCPredFiltering( Int* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight )
{
  Pel* pDst = rpDst;
  Int x, y, iDstStride2, iSrcStride2;

  // boundary pixels processing
  pDst[0] = (Pel)((pSrc[-iSrcStride] + pSrc[-1] + 2 * pDst[0] + 2) >> 2);

  for ( x = 1; x < iWidth; x++ )
  {
    pDst[x] = (Pel)((pSrc[x - iSrcStride] +  3 * pDst[x] + 2) >> 2);
  }

  for ( y = 1, iDstStride2 = iDstStride, iSrcStride2 = iSrcStride-1; y < iHeight; y++, iDstStride2+=iDstStride, iSrcStride2+=iSrcStride )
  {
    pDst[iDstStride2] = (Pel)((pSrc[iSrcStride2] + 3 * pDst[iDstStride2] + 2) >> 2);
  }

  return;
}
//! \}
