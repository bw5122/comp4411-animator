#include "BsplineCurveEvaluator.h"
#include "BezierCurveEvaluator.h"
#include <assert.h>
#include "mat.h"
#include "vec.h"
#include "modelerapp.h"

//#define SEGMENT 30

void BsplineCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const
{
	//ptvEvaluatedCurvePts.clear();
	float scale = 100;
	int iCtrlPtCount = ptvCtrlPts.size();

	std::vector<Point> pts;
	ptvEvaluatedCurvePts.clear();
	if (bWrap == false) {
		//initialize control points to ensure end points
		for (int i = 0; i < 3; i++)
			pts.push_back(ptvCtrlPts[0]);
		for (int i = 0; i < ptvCtrlPts.size(); i++)
			pts.push_back(ptvCtrlPts[i]);
		for (int i = 0; i < 3; i++)
			pts.push_back(ptvCtrlPts[ptvCtrlPts.size() - 1]);
		iCtrlPtCount = pts.size();


		//construct b-spline curve
		for (int i = 0; i < iCtrlPtCount - 3; i++) {
			if (i == 0 || i == iCtrlPtCount - 4) {
				ptvEvaluatedCurvePts.push_back(pts[i]);
				continue;
			}

			double ax = (-1 * pts[i].x + 3 * pts[i + 1].x - 3 * pts[i + 2].x + pts[i + 3].x) / 6;
			double bx = (3 * pts[i].x - 6 * pts[i + 1].x + 3 * pts[i + 2].x) / 6;
			double cx = (-3 * pts[i].x + 3 * pts[i + 2].x) / 6;
			double dx = (pts[i].x + 4 * pts[i + 1].x + pts[i + 2].x) / 6;
			double ay = (-1 * pts[i].y + 3 * pts[i + 1].y - 3 * pts[i + 2].y + pts[i + 3].y) / 6;
			double by = (3 * pts[i].y - 6 * pts[i + 1].y + 3 * pts[i + 2].y) / 6;
			double cy = (-3 * pts[i].y + 3 * pts[i + 2].y) / 6;
			double dy = (pts[i].y + 4 * pts[i + 1].y + pts[i + 2].y) / 6;

			for (double t = 0; t <= 1.0; t += 1.0 / scale) {
				double x = ax * t*t*t + bx * t*t + cx * t + dx;
				double y = ay * t*t*t + by * t*t + cy * t + dy;
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
		//initialize control points
		if (iCtrlPtCount > 2) {
			for (int i = 0; i < iCtrlPtCount; i++)
				pts.push_back(ptvCtrlPts[i]);
			for (int i = 0; i < 3; i++)
				pts.push_back(Point(ptvCtrlPts[i].x + fAniLength, ptvCtrlPts[i].y));
		}
		else {
			for (int i = 0; i < iCtrlPtCount; i++)
				pts.push_back(ptvCtrlPts[i]);
			for (int i = 0; i < 2; i++)
				pts.push_back(Point(ptvCtrlPts[i].x + fAniLength, ptvCtrlPts[i].y));
			pts.push_back(Point(ptvCtrlPts[0].x + 2 * fAniLength, ptvCtrlPts[0].y));
		}
		iCtrlPtCount = pts.size();
		//construct the curve
		for (int i = 0; i < iCtrlPtCount - 3; i++) {

			double ax = (-1 * pts[i].x + 3 * pts[i + 1].x - 3 * pts[i + 2].x + pts[i + 3].x) / 6;
			double bx = (3 * pts[i].x - 6 * pts[i + 1].x + 3 * pts[i + 2].x) / 6;
			double cx = (-3 * pts[i].x + 3 * pts[i + 2].x) / 6;
			double dx = (pts[i].x + 4 * pts[i + 1].x + pts[i + 2].x) / 6;
			double ay = (-1 * pts[i].y + 3 * pts[i + 1].y - 3 * pts[i + 2].y + pts[i + 3].y) / 6;
			double by = (3 * pts[i].y - 6 * pts[i + 1].y + 3 * pts[i + 2].y) / 6;
			double cy = (-3 * pts[i].y + 3 * pts[i + 2].y) / 6;
			double dy = (pts[i].y + 4 * pts[i + 1].y + pts[i + 2].y) / 6;

			for (double t = 0; t <= 1.0; t += 1.0 / scale) {
				double x = ax * t*t*t + bx * t*t + cx * t + dx;
				double y = ay * t*t*t + by * t*t + cy * t + dy;
				if (x > fAniLength)
					x -= fAniLength;
				ptvEvaluatedCurvePts.push_back(Point(x, y));
			}
		}

	}
}
