#include "CRCurveEvaluator.h"

#include <assert.h>
#include "mat.h"
#include "vec.h"
#include "modelerapp.h"
#include "modelerui.h"
void CRCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const
{
	//ModelerUI* pUI = ModelerApplication::Instance()->getUI();
	double tension = 0.5;
		//pUI->Tension->value();
	float scale = 100;
	int iCtrlPtCount = ptvCtrlPts.size();

	std::vector<Point> pts;
	ptvEvaluatedCurvePts.clear();
	if (bWrap == false) {
		//initialze control points
		pts.push_back(ptvCtrlPts[0]);
		for (int i = 0; i < ptvCtrlPts.size(); i++)
			pts.push_back(ptvCtrlPts[i]);
		pts.push_back(ptvCtrlPts[ptvCtrlPts.size() - 1]);
		iCtrlPtCount = pts.size();
		for (int i = 0; i < iCtrlPtCount - 3; i++) {
			/*if (pts[i + 2].x - pts[i + 1].x < 1.0) {
				ptvEvaluatedCurvePts.push_back(pts[i + 1]);
				ptvEvaluatedCurvePts.push_back(pts[i + 2]);
				continue;
			}*/
			double ax = -(tension)* pts[i].x + (-tension + 2) * pts[i + 1].x + (tension - 2) * pts[i + 2].x + tension * pts[i + 3].x;
			double bx = (2 * tension * pts[i].x + (tension - 3) * pts[i + 1].x + (-2 * tension + 3) * pts[i + 2].x - tension * pts[i + 3].x);
			double cx = (-tension * pts[i].x + tension * pts[i + 2].x);
			double dx = pts[i + 1].x;


			double ay = -(tension)* pts[i].y + (-tension + 2) * pts[i + 1].y + (tension - 2) * pts[i + 2].y + tension * pts[i + 3].y;
			double by = (2 * tension * pts[i].y + (tension - 3) * pts[i + 1].y + (-2 * tension + 3) * pts[i + 2].y - tension * pts[i + 3].y);
			double cy = (-tension * pts[i].y + tension * pts[i + 2].y);
			double dy = pts[i + 1].y;


			for (double t = 0; t <= 1.0; t += 1.0 / scale) {
				double x = ax * t*t*t + bx * t*t + cx * t + dx;
				double y = ay * t*t*t + by * t*t + cy * t + dy;
				if (!ptvEvaluatedCurvePts.empty()) {
					if (ptvEvaluatedCurvePts.back().x >= x)
						continue;
				}
				ptvEvaluatedCurvePts.push_back(Point(x, y));
			}

		}
		double x1 = 0.0;
		double x2 = fAniLength;
		double y1 = ptvCtrlPts[0].y;
		double y2 = ptvCtrlPts[ptvCtrlPts.size() - 1].y;
		ptvEvaluatedCurvePts.push_back(Point(x1, y1));
		ptvEvaluatedCurvePts.push_back(Point(x2, y2));
	}
	else {
		//initialize the control points
		if (iCtrlPtCount == 2) {
			for (int i = 0; i < iCtrlPtCount; i++)
				pts.push_back(ptvCtrlPts[i]);
			for (int i = 0; i < iCtrlPtCount; i++)
				pts.push_back(Point(ptvCtrlPts[i].x + fAniLength, ptvCtrlPts[i].y));
			pts.push_back(Point(ptvCtrlPts[0].x + 2 * fAniLength, ptvCtrlPts[0].y));
		}
		else {
			for (int i = 0; i < iCtrlPtCount; i++)
				pts.push_back(ptvCtrlPts[i]);
			for (int i = 0; i < 3; i++)
				pts.push_back(Point(ptvCtrlPts[i].x + fAniLength, ptvCtrlPts[i].y));
		}
		iCtrlPtCount = pts.size();
		for (int i = 0; i < iCtrlPtCount - 3; i++) {
			/*if (pts[i + 2].x - pts[i + 1].x < 1.0) {
				ptvEvaluatedCurvePts.push_back(pts[i + 1]);
				ptvEvaluatedCurvePts.push_back(pts[i + 2]);
				continue;
			}*/

			double ax = -(tension)* pts[i].x + (-tension + 2) * pts[i + 1].x + (tension - 2) * pts[i + 2].x + tension * pts[i + 3].x;
			double bx = (2 * tension * pts[i].x + (tension - 3) * pts[i + 1].x + (-2 * tension + 3) * pts[i + 2].x - tension * pts[i + 3].x);
			double cx = (-tension * pts[i].x + tension * pts[i + 2].x);
			double dx = pts[i + 1].x;


			double ay = -(tension)* pts[i].y + (-tension + 2) * pts[i + 1].y + (tension - 2) * pts[i + 2].y + tension * pts[i + 3].y;
			double by = (2 * tension * pts[i].y + (tension - 3) * pts[i + 1].y + (-2 * tension + 3) * pts[i + 2].y - tension * pts[i + 3].y);
			double cy = (-tension * pts[i].y + tension * pts[i + 2].y);
			double dy = pts[i + 1].y;

			//double ax = (-1 * pts[i].x + 3 * pts[i + 1].x - 3 * pts[i + 2].x + pts[i + 3].x) / 2;
			//double bx = (2 * pts[i].x - 5 * pts[i + 1].x + 4 * pts[i + 2].x - pts[i + 3].x) / 2;
			//double cx = (-1 * pts[i].x + pts[i + 2].x) / 2;
			//double dx = 2 * pts[i + 1].x / 2;
			//double ay = (-1 * pts[i].y + 3 * pts[i + 1].y - 3 * pts[i + 2].y + pts[i + 3].y) / 2;
			//double by = (2 * pts[i].y - 5 * pts[i + 1].y + 4 * pts[i + 2].y - pts[i + 3].y) / 2;
			//double cy = (-1 * pts[i].y + pts[i + 2].y) / 2;
			//double dy = 2 * pts[i + 1].y / 2;
			for (double t = 0; t <= 1.0; t += 1.0 / scale) {
				double x = ax * t*t*t + bx * t*t + cx * t + dx;
				double y = ay * t*t*t + by * t*t + cy * t + dy;
				if (!ptvEvaluatedCurvePts.empty()) {
					if (ptvEvaluatedCurvePts.back().x >= x)
						continue;
				}
				if (x > fAniLength)
					x -= fAniLength;
				ptvEvaluatedCurvePts.push_back(Point(x, y));
			}
		}
	}

	return;
}