#include <iostream>
#include <cmath>
#include <unordered_set>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <rlutil.h>
#include "AStar.hpp"

const int Width = 60, Height = 40, CellSize = 15;

sf::Color BgColor(0, 0, 0), //PathLineColor(255, 255, 0),
	PathColor(0, 170, 0), SearchColor(10, 30, 10),
	StartColor(200, 0, 0), GoalColor(50, 50, 255),
	WallColor(200, 200, 200);

sf::RenderWindow Win;
sf::RenderTexture RenderTex;

std::vector<Coordinate> PathVector;

AStar AStarMap;
Coordinate Start(0, 0), Goal(Width-1, Height-1);

extern void Initialization();
extern void UpdateMaps();
extern void RenderMap();

int main()
{
	std::ios::sync_with_stdio(false);

	Initialization();
	UpdateMaps();

	sf::Event event;
	Coordinate mousePos;

	bool mouseDown = false, moveStart=false, moveGoal=false;
	sf::Mouse::Button mouseButton;
	while(Win.isOpen())
	{
		bool hasAction = false;
		while(Win.pollEvent(event))
		{
			if(event.type == sf::Event::Closed)
				Win.close();
			if(event.type == sf::Event::MouseButtonPressed)
			{
				Win.setFramerateLimit(0);
	 			mouseDown = true, mouseButton = event.mouseButton.button;

				moveStart = false, moveGoal = false;
				if(mousePos == Start)
					moveStart = true;
				else if(mousePos == Goal)
					moveGoal = true;
			}
			if(event.type == sf::Event::MouseButtonReleased)
			{
				Win.setFramerateLimit(60);
				mouseDown = false, moveStart = false, moveGoal = false;
			}
			if(event.type == sf::Event::MouseMoved)
			{
				mousePos.x = event.mouseMove.x / CellSize,
					mousePos.y = event.mouseMove.y / CellSize;
				if(mousePos.x < 0) mousePos.x = 0;
				if(mousePos.y < 0) mousePos.y = 0;
				if(mousePos.x >= Width) mousePos.x = Width - 1;
				if(mousePos.y >= Height) mousePos.y = Height - 1;
			}
		}
		if(mouseDown)
		{
			if(mouseButton == sf::Mouse::Left 
					&& !AStarMap.IsWall(mousePos)
					&& mousePos != Start && mousePos != Goal)
			{
				if(moveStart)// move start
				{
					Start = mousePos;
					hasAction = true;
				}
				else if(moveGoal)// move goal
				{
					Goal = mousePos;
					hasAction = true;
				}
				else if(!moveStart && !moveGoal)// set wall
				{
					AStarMap.SetWall(mousePos, true);
					hasAction = true;
				}
			}
			else if(mouseButton == sf::Mouse::Right
					&& AStarMap.IsWall(mousePos))// erase wall
			{
				AStarMap.SetWall(mousePos, false);
				hasAction = true;
			}
		}
		if(hasAction)
			UpdateMaps();
		RenderMap();
		Win.display();
	}
	return EXIT_SUCCESS;
}
double ManhattanAStarHeuristic(const Coordinate &now, const Coordinate &goal)
{
	int dx = abs(now.x - goal.x);
	int dy = abs(now.y - goal.y);
	return dx + dy;
}
std::list<Coordinate> ManhattanAStarNeighbours(const Coordinate &now, const AStar &astar)
{
	static std::vector<Coordinate> dirs = {{0, 1, 1}, {0, -1, 1}, {1, 0, 1}, {-1, 0, 1}};
	static std::vector<Coordinate> reversed_dirs = {{-1, 0, 1}, {1, 0, 1}, {0, -1, 1}, {0, 1, 1}};
	static bool flip;
	if(now == Start)
		flip = true;
	else 
		flip = !flip;
	std::list<Coordinate> return_v;
	for(size_t i=0; i<dirs.size(); ++i)
	{
		const Coordinate &d = flip ? dirs[i] : reversed_dirs[i];
		Coordinate nei(d.x + now.x, d.y + now.y, d.eval);
		if(astar.IsWall(nei))
			continue;

		return_v.push_back(nei);
	}
	return return_v;
}
const double DiagonalDist = 1.4142;
double DiagonalAStarHeuristic(const Coordinate &now, const Coordinate &goal)
{
	int dx = abs(now.x - goal.x);
	int dy = abs(now.y - goal.y);
	//return dx + dy;

	if(dx > dy)
		return DiagonalDist * dy + (dx - dy);
	else
		return DiagonalDist * dx + (dy - dx);
}
std::list<Coordinate> DiagonalAStarNeighbours(const Coordinate &now, const AStar &astar)
{
	static std::list<Coordinate> dirs = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};
	static std::list<Coordinate> dia_dirs = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
	std::list<Coordinate> return_v;
	for(const Coordinate &d : dirs)
	{
		Coordinate nei(d.x + now.x, d.y + now.y, 1);
		if(astar.IsWall(nei))
			continue;

		return_v.push_back(nei);
	}
	for(const Coordinate &d : dia_dirs)
	{
		Coordinate dirx(d.x + now.x, now.y), diry(now.x, d.y + now.y);
		if(astar.IsWall(dirx) && astar.IsWall(diry))
			continue;

		Coordinate nei(d.x + now.x, d.y + now.y, DiagonalDist);
		if(astar.IsWall(nei))
			continue;

		return_v.push_back(nei);
	}
	return return_v;
}

void Initialization()
{
	Win.create(sf::VideoMode(Width * CellSize, Height * CellSize), 
			"AStar Algorithm Demo", sf::Style::Titlebar | sf::Style::Close);
	RenderTex.create(Width * CellSize, Height * CellSize, false);
	AStarMap.SetSize(Width, Height);

	//AStarMap.SetHeuristicFunction(ManhattanAStarHeuristic);
	//AStarMap.SetNeighbourFunction(ManhattanAStarNeighbours);
	AStarMap.SetHeuristicFunction(DiagonalAStarHeuristic);
	AStarMap.SetNeighbourFunction(DiagonalAStarNeighbours);
}

void UpdateMaps()
{
	PathVector.clear();

	AStarMap.DoAStart(Start, Goal, PathVector);

	//redraw the RenderTexture
	RenderTex.clear(BgColor);
	sf::RectangleShape cell(sf::Vector2f(CellSize, CellSize));
	Coordinate i;
	for(i.x=0; i.x<Width; ++i.x)
		for(i.y=0; i.y<Height; ++i.y)
		{
			sf::Color cellColor;
			if(i == Start)
				cellColor = StartColor;
			else if(i == Goal)
				cellColor = GoalColor;
			else if(AStarMap.IsPath(i))
				cellColor = PathColor;
			else if(AStarMap.IsSearched(i))
				cellColor = SearchColor;
			else if(AStarMap.IsWall(i))
				cellColor = WallColor;
			else continue;

			cell.setFillColor(cellColor);
			cell.setPosition(i.x * CellSize, i.y * CellSize);

			RenderTex.draw(cell);
		}
	/*sf::VertexArray pathLine(sf::LinesStrip, PathVector.size() + 1);

	  pathLine[0].position = sf::Vector2f(
	  Start.x * CellSize + CellSize/2, 
	  Start.y * CellSize + CellSize/2);
	  pathLine[0].color = PathLineColor;
	  for(size_t i=0; i<PathVector.size(); ++i)
	  {
	  pathLine[i+1].position = sf::Vector2f(
	  PathVector[i].x * CellSize + CellSize/2, 
	  PathVector[i].y * CellSize + CellSize/2);
	  pathLine[i+1].color = PathLineColor;
	  }

	  RenderTex.draw(pathLine);*/

	RenderTex.display();

	rlutil::cls();
	std::cout << "searched: \t" << AStarMap.GetSearcheNum() << " / " << Width * Height << std::endl
		<< "distance: \t" << (PathVector.empty() ? 
				std::numeric_limits<double>::infinity() : PathVector.back().eval) << std::endl
		<< "walk steps: \t" << PathVector.size() << std::endl
		<< "spend time: \t" << AStarMap.GetLastedTime() * 1000.0 << " ms" << std::endl;
}

void RenderMap()
{
	const sf::Texture& texture = RenderTex.getTexture();

	Win.draw(sf::Sprite(texture));
}
