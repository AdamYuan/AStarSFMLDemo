#pragma once
#include <numeric>
#include <functional>
#include <algorithm>
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
		clock_t Time;
		heuFunction heuristicFunc;
		neighbourFunction neighboursFunc;
		int Width = 0, Height = 0;
		bool **WallMap;
		bool **SearchMap;
		bool **PathMap;
		int SearchNum;
		static bool CompareInPriorityQueue (const Coordinate &n1, const Coordinate &n2)
		{
			return n1.eval > n2.eval;
		}
		void AllocArrays()
		{
			WallMap = new bool*[Width];
			PathMap = new bool*[Width];
			SearchMap = new bool*[Width];
			for(int i=0; i<Width; ++i)
			{
				WallMap[i] = new bool[Height];
				for(int j=0; j<Height; ++j)
					WallMap[i][j] = false;
				PathMap[i] = new bool[Height];
				SearchMap[i] = new bool[Height];
			}
		}
		void FreeArrays()
		{
			for(int i=0; i<Width; ++i)
			{
				delete [] WallMap[i];
				delete [] PathMap[i];
				delete [] SearchMap[i];
			}
			delete [] WallMap;
			delete [] PathMap;
			delete [] SearchMap;
		}

	public:

		AStar() = default;
		void SetSize(int w, int h)
		{
			FreeArrays();
			Width = w, Height = h;
			AllocArrays();
		}
		~AStar()
		{
			FreeArrays();
		}
		Coordinate GetSize() const
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
			return WallMap[pos.x][pos.y];
		}
		void SetWall(const Coordinate &pos, bool wall)
		{
			if(pos.x < 0 || pos.x >= Width || pos.y < 0 || pos.y >= Height)
				return;
			WallMap[pos.x][pos.y] = wall;
		}
		bool IsSearched(const Coordinate &pos) const
		{
			return SearchMap[pos.x][pos.y];
		}
		bool IsPath(const Coordinate &pos) const
		{
			return PathMap[pos.x][pos.y];
		}
		int GetSearcheNum() const
		{
			return SearchNum;
		}
		double GetLastedTime() const
		{
			return Time / (double) CLOCKS_PER_SEC;
		}

		bool DoAStart(const Coordinate &start, const Coordinate &goal, std::vector<Coordinate> &path)
		{
			Time = clock();

			std::priority_queue<Coordinate, std::vector<Coordinate>, decltype(&CompareInPriorityQueue)> 
				openSet(&CompareInPriorityQueue);
			bool closeSet [Width][Height];

			Coordinate cameFrom [Width][Height];
			double gScore [Width][Height];

			SearchNum = 1;
			for(int i=0; i<Width; ++i)
				for(int j=0; j<Height; ++j)
				{
					SearchMap[i][j] = false;
					PathMap[i][j] = false;
					gScore[i][j] = std::numeric_limits<double>::max();
					closeSet[i][j] = false;
				}

#define V(array2d, Coordinate) (array2d[(Coordinate).x][(Coordinate).y])
#define P 0.00001

			Coordinate first = start;
			V(SearchMap, first) = true;
			V(gScore, first) = 0;
			first.eval = 0 + heuristicFunc(first, goal) * (1 + P);

			openSet.push(first);

			bool returnV = false;
			while(!openSet.empty())
			{
				Coordinate now = openSet.top();

				if(now == goal)
				{
					Coordinate &p = now;
					while(p != start)
					{
						V(PathMap, p) = true;
						path.push_back(p);
						p = V(cameFrom, p);
					}
					std::reverse(path.begin(), path.end());
					returnV = true;
					break;
				}

				openSet.pop();

				if(V(closeSet, now))
					continue;

				V(closeSet, now) = true;

				for(Coordinate &nei : neighboursFunc(now, *this))
				{	
					double tentative_gScore = V(gScore, now) + nei.eval;

					if(V(closeSet, nei) || tentative_gScore >= V(gScore, nei))
						continue;

					if(!V(SearchMap, nei))
					{
						SearchNum ++;
						V(SearchMap, nei) = true;
					}
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
