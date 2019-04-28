#pragma once
#ifndef INCLUDED_CR_CURVE_EVALUATOR_H
#define INCLUDED_CR_CURVE_EVALUATOR_H


#include "CurveEvaluator.h"
#include "vec.h"

//using namespace std;

class CRCurveEvaluator : public CurveEvaluator
{
public:
	void evaluateCurve(const std::vector<Point>& ptvCtrlPts,
		std::vector<Point>& ptvEvaluatedCurvePts,
		const float& fAniLength,
		const bool& bWrap) const;
	void DisplayBezier(const Point& p1, const Point& p2, const Point& p3, const Point& p4, std::vector<Point>& ptvEvaluatedCurvePts, const float& fAniLength) const;
};

#endif