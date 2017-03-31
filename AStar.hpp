#pragma once
#include <numeric>
#include <functional>
#include <algorithm>
#include <unordered_set>
#include <queue>
#include <vector>
#include <list>

struct Coordinate
{
	int x, y;
	double eval;
	Coordinate(int _x = 0, int _y = 0, double _e = std::numeric_limits<double>::max())
		: x(_x), y(_y), eval(_e){}
	bool operator == (const Coordinate &n) const
	{
		return x == n.x && y == n.y;
	}
	bool operator != (const Coordinate &n) const
	{
		return x != n.x || y != n.y;
	}
};

namespace std {
	template <>
		struct hash<Coordinate>
		{
			std::size_t operator()(const Coordinate& k) const
			{
				//return k.x * 65536 + k.y;
				return std::hash<int>()(k.x) ^ std::hash<int>()(k.y);
			}
		};
}

class AStar;
typedef std::function<double(const Coordinate&, const Coordinate&)> heuFunction;
typedef std::function<std::list<Coordinate>(const Coordinate&, const AStar&)> neighbourFunction;
class AStar
{
	private:
		std::unordered_set<Coordinate> WallSet;
		std::unordered_set<Coordinate> SearchSet;
		clock_t Time;
		heuFunction heuristicFunc;
		neighbourFunction neighboursFunc;
		static bool CompareInPriorityQueue (const Coordinate &n1, const Coordinate &n2)
		{
			return n1.eval > n2.eval;
		}

		int Width, Height;

	public:

		AStar() = default;
		AStar(int w, int h) : Width(w), Height(h){}
		void SetSize(int w, int h)
		{
			Width = w, Height = h;
		}
		Coordinate GetSize()
		{
			return Coordinate(Width, Height);
		}
		void SetHeuristicFunction(const heuFunction &heu)
		{
			heuristicFunc = heu;
		}
		void SetNeighbourFunction(const neighbourFunction &nei)
		{
			neighboursFunc = nei;
		}
		bool IsWall(const Coordinate &pos) const
		{
			if(pos.x < 0 || pos.x >= Width || pos.y < 0 || pos.y >= Height)
				return true;
			return WallSet.count(pos);
		}
		void SetWall(const Coordinate &pos, bool wall)
		{
			if(wall)
				WallSet.insert(pos);
			else
				WallSet.erase(pos);
		}
		const decltype(SearchSet) &GetSearchSet()
		{
			return SearchSet;
		}
		double GetLastedTime()
		{
			return Time / (double) CLOCKS_PER_SEC;
		}

		bool DoAStart(const Coordinate &start, const Coordinate &goal, std::vector<Coordinate> &path)
		{
			Time = clock();

			SearchSet.clear();
			SearchSet.insert(start);

			std::priority_queue<Coordinate, std::deque<Coordinate>, decltype(&CompareInPriorityQueue)> 
				openSet(&CompareInPriorityQueue);
			std::unordered_set<Coordinate> closeSet;
			
			Coordinate cameFrom [Width][Height];
			double gScore [Width][Height];

			for(int i=0; i<Width; ++i)
				for(int j=0; j<Height; ++j)
				{
					gScore[i][j] = std::numeric_limits<double>::max();
				}

#define V(array2d, Coordinate) (array2d[(Coordinate).x][(Coordinate).y])
#define P 0.00001

			Coordinate first = start;
			V(gScore, first) = 0;
			first.eval = 0 + heuristicFunc(first, goal) * (1 + P);

			openSet.push(first);

			bool returnV = false;
			while(!openSet.empty())
			{
				Coordinate now = openSet.top();

				if(now == goal)
				{
					Coordinate p = now;
					while(p != start)
					{
						path.push_back(p);
						p = V(cameFrom, p);
					}
					std::reverse(path.begin(), path.end());
					returnV = true;
					break;
				}

				openSet.pop();

				if(closeSet.count(now))
					continue;

				closeSet.insert(now);

				for(Coordinate &nei : neighboursFunc(now, *this))
				{	
					double tentative_gScore = V(gScore, now) + nei.eval;

					if(closeSet.count(nei) || tentative_gScore >= V(gScore, nei))
						continue;


					SearchSet.insert(nei);

					V(cameFrom, nei) = now;
					V(gScore, nei) = tentative_gScore;
					nei.eval = V(gScore, nei) + heuristicFunc(nei, goal) * (1 + P);

					openSet.push(nei);
				}
			}
			Time = clock() - Time;

			return returnV;
		}
};
