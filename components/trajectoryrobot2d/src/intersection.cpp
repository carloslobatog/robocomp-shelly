#include <intersection.h>


LineSegment getIntersection(LineSegment a, LineSegment b)
{
	/* the intersection [(x1,y1), (x2, y2)]
	it might be a line or a single point. If it is a line,
	then x1 = x2 and y1 = y2.  */
	float x1, y1, x2, y2;

	if (a.first.x == a.second.x)
	{
		// Case (A)
		// As a is a perfect vertical line, it cannot be represented
		// nicely in a mathematical way. But we directly know that
		//
		x1 = a.first.x;
		x2 = x1;
		if (b.first.x == b.second.x)
		{
			// Case (AA): all x are the same!
			// Normalize
			if (a.first.y > a.second.y)
			{
				std::swap(a.second, a.first);
			}
			if (b.first.y > b.second.y)
			{
				std::swap(b.second, b.first);
			}
			if (a.first.y > b.first.y)
			{
				std::swap(a, b);
			}

			// Now we know that the y-value of a.first is the 
			// lowest of all 4 y values
			// this means, we are either in case (AAA):
			//   a: x--------------x
			//   b:	x---------------x
			// or in case (AAB)
			//   a: x--------------x
			//   b:	x-------x
			// in both cases:
			// get the relavant y intervall
			y1 = b.first.y;
			y2 = std::min(a.second.y, b.second.y);
		}
		else
		{
			// Case (AB)
			// we can mathematically represent line b as
			//	 y = m*x + t <=> t = y - m*x
			// m = (y1-y2)/(x1-x2)
			float m, t;
			m = (b.first.y - b.second.y)  /  (b.first.x - b.second.x);
			t = b.first.y - m*b.first.x;
			y1 = m*x1 + t;
			y2 = y1;
		}
	}
	else if (b.first.x == b.second.x)
	{
		// Case (B)
		// essentially the same as Case (AB), but with
		// a and b switched
		x1 = b.first.x;
		x2 = x1;

		std::swap(a, b);

		float m, t;
		m = (b.first.y - b.second.y)   /  (b.first.x - b.second.x);
		t = b.first.y - m*b.first.x;
		y1 = m*x1 + t;
		y2 = y1;
	}
	else
	{
		// Case (C)
		// Both lines can be represented mathematically
		float ma, mb, ta, tb;
		ma = (a.first.y - a.second.y)  /  (a.first.x - a.second.x);
		mb = (b.first.y - b.second.y)  /  (b.first.x - b.second.x);
		ta = a.first.y - ma*a.first.x;
		tb = b.first.y - mb*b.first.x;
		if (ma == mb)
		{
			// Case (CA)
			// both lines are in parallel. As we know that they 
			// intersect, the intersection could be a line
			// when we rotated this, it would be the same situation 
			// as in case (AA)

			// Normalize
			if(a.first.x > a.second.x) {
				std::swap(a.second, a.first);
			}
			if(b.first.x > b.second.x) {
				std::swap(b.second, b.first);
			}
			if(a.first.x > b.first.x) {
				std::swap(a, b);
			}

			// get the relavant x intervall
			x1 = b.first.x;
			x2 = std::min(a.second.x, b.second.x);
			y1 = ma*x1+ta;
			y2 = ma*x2+ta;
		}
		else
		{
			// Case (CB): only a point as intersection:
			// y = ma*x+ta
			// y = mb*x+tb
			// ma*x + ta = mb*x + tb
			// (ma-mb)*x = tb - ta
			// x = (tb - ta)/(ma-mb)
			x1 = (tb-ta)/(ma-mb);
			y1 = ma*x1+ta;
			x2 = x1;
			y2 = y1;
		}
	}
	
	return LineSegment(Pointinter(x1,y1), Pointinter(x2, y2));
}

