#pragma once
#include <tuple>
#include "olcPixelGameEngine.h"

// std::vector of Vector2Ds
#define Points std::vector<olc::vf2d>

inline bool operator==(const olc::vf2d& v1, const olc::vf2d& v2) { return v1.x == v2.x && v1.y == v2.y; }
inline bool operator!=(const olc::vf2d& v1, const olc::vf2d& v2) { return !(v1 == v2); }


///////////////////////////////////////////////// S H A P E S ///////////////////////////////////////////////


struct Line
{
	olc::vf2d p1;
	olc::vf2d p2;
};

struct Segment
{
	olc::vf2d p1;
	olc::vf2d p2;

	Line AsLine() const { return Line{ p1, p2 }; }
};

inline bool operator==(const Segment& s1, const Segment& s2) { return { s1.p1 == s2.p1 && s1.p2 == s2.p2 }; }
inline bool operator!=(const Segment& s1, const Segment& s2) { return !(s1 == s2); }

struct Ray
{
	olc::vf2d pos;
	olc::vf2d dir;
	Line AsLine() const { return Line{ pos, dir }; }
	Segment AsSegment() const { return Segment{ pos, (dir - pos) * 1000 }; }
};

struct Circle
{
	olc::vf2d pos;
	float radius;
};

struct Polygon
{
	Points points;
	olc::vf2d PointAfter(int i) const
	{
		return (i < points.size() - 1) ? points[i + 1] : points[0];
	}
};

// Axis-aligned Bounding Box
struct RectAABB
{
	olc::vf2d pos;
	olc::vf2d size;
	Polygon AsPolygon() const
	{
		return  Polygon{
			Points{
				pos,
				{pos.x + size.x, pos.y},
				pos + size,
				{pos.x, pos.y + size.y}
			}
		};
	}
};


/////////////////////////////////////////////// U T I L I T Y //////////////////////////////////////////////

template <typename T>
bool IsBetween(T f, T low, T high)
{
	return f > low && f < high;
}

// Returns whether an intersection is within the bounds of a line segment
bool IsLegalSegment(const olc::vf2d& i, const Segment& s)
{
	olc::vf2d check = {
		(i.x - s.p1.x) / (s.p2.x - s.p1.x),
		(i.y - s.p1.y) / (s.p2.y - s.p1.y)
	};

	return (IsBetween(check.x, 0.0f, 1.0f) || IsBetween(check.y, 0.0f, 1.0f));
}

// used in calculations
struct StandardForm
{
	float A, B, C;
};

// Returns the standard form of a line
StandardForm GetStandardForm(const Line& l)
{
	float A = l.p2.y - l.p1.y;
	float B = l.p1.x - l.p2.x;
	float C = A * l.p1.x + B * l.p1.y;
	return { A, B, C };
}

// Takes a list of intersections and returns a vector containing those which are
// within the bounds of either one or two line segments. Used frequently in other functions.
const Segment NULL_SEG = { {0.0f, 0.0f}, {0.0f, 0.0f} };
Points SegmentIntersections(Points intersections, const Segment& seg1, const Segment& seg2 = NULL_SEG)
{
	Points result(0);
	if (intersections.size() > 0)
		for (olc::vf2d p : intersections)
		{
			if (IsLegalSegment(p, seg1))
			{
				if (seg2 == NULL_SEG)
				{
					result.push_back(p);
				}
				else
				{
					if (IsLegalSegment(p, seg2))
						result.push_back(p);
				}
			}
		}

	return result;
}

// The equation for the intersection between a circle and a line in standard form. 
// Don't forget to translate to the origin before and after using!
Points CircleVsStandardLine(const Circle& c, const StandardForm& s)
{
	// copied and pasted - could put in seperate function
	float fA2byB2 = (s.A * s.A) + (s.B * s.B);
	if (fA2byB2 == 0) return Points(0);

	float fSqrt = (c.radius * c.radius) * (fA2byB2) - (s.C * s.C);
	if (fSqrt == 0) return Points(0);

	return Points{
		olc::vf2d{
			((s.A * s.C) + s.B * sqrt(fSqrt)) / fA2byB2,
			((s.B * s.C) - s.A * sqrt(fSqrt)) / fA2byB2
		} + c.pos,	// translate back

		olc::vf2d{
			((s.A * s.C) - s.B * sqrt(fSqrt)) / fA2byB2,
			((s.B * s.C) + s.A * sqrt(fSqrt)) / fA2byB2
		} + c.pos
	};
}


/////////////////////////////////////////////// I N T E R S E C T I O N S ///////////////////////////////////////////////


inline bool PointInCircle(const olc::vf2d& p, const Circle& c)
{
	return (c.pos - p).mag() < c.radius;
}

inline bool PointInRect(const olc::vf2d& p, const RectAABB& r)
{
	return (
		p.x > r.pos.x &&
		p.y > r.pos.y &&
		p.x < r.pos.x + r.size.x &&
		p.y < r.pos.y + r.size.y);
}


//////////////////////// L I N E   V S . ////////////////////////////


inline Points LineVsLine(const Line& l1, const Line& l2)
{
	StandardForm s1 = GetStandardForm(l1);
	StandardForm s2 = GetStandardForm(l2);
	
	float fDenom = s1.A * s2.B - s2.A * s1.B;
	if (fDenom == 0) return Points(0);

	return Points{ 
		olc::vf2d{
			(s2.B * s1.C - s1.B * s2.C) / fDenom,
			(s2.C * s1.A - s1.C * s2.A) / fDenom
		}
	};
}

inline Points LineVsSegment(const Line& l, const Segment& s)
{
	return SegmentIntersections(LineVsLine(l, s.AsLine()), s);
}

inline Points LineVsRay(const Line& l, const Ray& r)
{
	return SegmentIntersections(LineVsLine(l, r.AsLine()), r.AsSegment());
}

inline Points LineVsCircle(const Line& l, const Circle& c)
{
	Line t = {
		l.p1 - c.pos,
		l.p2 - c.pos };

	Points intersections = CircleVsStandardLine(c, GetStandardForm(t));
	for (olc::vf2d p : intersections)
		p += c.pos;

	return intersections;
}

inline Points LineVsPolygon(const Line& l, const Polygon& p)
{
	Points result;
	for (int i = 0; i < p.points.size(); i++)
	{
		Points intersections = LineVsSegment(l, Segment{ p.points[i], p.PointAfter(i) });
		result.insert(result.begin(), intersections.begin(), intersections.end());
	}
	return result;
}

inline Points LineVsRectAABB(const Line& l, const RectAABB& r)
{
	return LineVsPolygon(l, r.AsPolygon());
}


//////////////////////// S E G M E N T   V S . ////////////////////////////


inline Points SegmentVsSegment(const Segment& s1, const Segment& s2)
{
	return SegmentIntersections(LineVsLine(s1.AsLine(), s2.AsLine()), s1, s2);
}

inline Points SegmentVsRay(const Segment& s, const Ray& r)
{
	return SegmentIntersections(LineVsLine(s.AsLine(), r.AsLine()), s, r.AsSegment());
}

inline Points SegmentVsCircle(const Segment& s, const Circle& c)
{
	return SegmentIntersections(LineVsCircle(s.AsLine(), c), s);
}

inline Points SegmentVsPolygon(const Segment& s, const Polygon& p)
{
	Points result;
	for (int i = 0; i < p.points.size(); i++)
	{
		Points intersections = SegmentVsSegment(s, Segment{ p.points[i], p.PointAfter(i) });
		result.insert(result.begin(), intersections.begin(), intersections.end());
	}
	return result;
}

inline Points SegmentVsRectAABB(const Segment& s, const RectAABB& r)
{
	return SegmentVsPolygon(s, r.AsPolygon());
}


/////////////////////////////// R A Y   V S . ////////////////////////////////////


inline Points RayVsRay(const Ray& r1, const Ray& r2)
{
	return SegmentIntersections(LineVsLine(r1.AsLine(), r2.AsLine()), r1.AsSegment(), r2.AsSegment());
}


inline Points RayVsCircle(const Ray& r, const Circle& c)
{
	return SegmentIntersections(SegmentVsCircle(r.AsSegment(), c), r.AsSegment());
}

inline Points RayVsPolygon(const Ray& r, const Polygon& p)
{
	Points result;
	for (int i = 0; i < p.points.size(); i++)
	{
		Points intersections = SegmentVsRay(Segment{ p.points[i], p.PointAfter(i) }, r);
		result.insert(result.begin(), intersections.begin(), intersections.end());
	}
	return result;
}

inline Points RayVsRectAABB(const Ray& r, const RectAABB& rect)
{
	return RayVsPolygon(r, rect.AsPolygon());
}


///////////////////////////// C I R C L E   V S . //////////////////////////////////


inline Points CircleVsCircle(const Circle& c1, const Circle& c2)
{
	olc::vf2d p1 = c1.pos - c1.pos;
	olc::vf2d p2 = c2.pos - c1.pos;
	float r1 = c1.radius;
	float r2 = c2.radius;

	StandardForm LineBetween = {
		2 * (p2.x - p1.x),
		2 * (p2.y - p1.y),
		(r1 * r1) - (p1.x * p1.x) - (p1.y * p1.y) - (r2 * r2) + (p2.x * p2.x) + (p2.y * p2.y)
	};

	Points intersections = CircleVsStandardLine(c1, LineBetween);
	for (olc::vf2d p : intersections)
		p += c1.pos;

	return intersections;
}

inline Points CircleVsPolygon(const Circle& c, const Polygon& p)
{
	Points result;
	for (int i = 0; i < p.points.size(); i++)
	{
		Points intersections = SegmentVsCircle(Segment{ p.points[i], p.PointAfter(i) }, c);
		result.insert(result.begin(), intersections.begin(), intersections.end());
	}
	return result;
}

inline Points CircleVsRectAABB(const Circle& c, const RectAABB& r)
{
	return CircleVsPolygon(c, r.AsPolygon());
}


//////////////////////////////// P O L Y G O N   V S . ////////////////////////////////////


inline Points PolygonVsPolygon(const Polygon& p1, const Polygon& p2)
{
	Points result;
	for (int i = 0; i < p1.points.size(); i++)
	{
		Points intersections = SegmentVsPolygon(Segment{ p1.points[i], p1.PointAfter(i) }, p2);
		result.insert(result.begin(), intersections.begin(), intersections.end());
	}
	return result;
}

inline Points PolygonVsRectAABB(const Polygon& p1, const RectAABB& r)
{
	return PolygonVsPolygon(p1, r.AsPolygon());
}


//////////////////////////////// R E C T   A A B B   V S . ///////////////////////////////////


inline Points RectAABBVsRectAABB(const RectAABB& r1, const RectAABB& r2)
{
	return PolygonVsPolygon(r1.AsPolygon(), r2.AsPolygon());
}