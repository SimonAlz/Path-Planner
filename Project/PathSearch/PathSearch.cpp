#include "PathSearch.h"
#include <math.h>
namespace fullsail_ai { namespace algorithms {

	PathSearch::PathSearch()
	{
		tileMap = nullptr;
	}

	PathSearch::~PathSearch()
	{
	}

	void PathSearch::initialize(TileMap* _tileMap)
	{
		tileMap = _tileMap;

		for (size_t i = 0; i < _tileMap->getRowCount(); i++)
		{
			for (size_t j = 0; j < _tileMap->getColumnCount(); j++)
			{
				if (_tileMap->getTile(i, j)->getWeight() == 0) 
					continue;
				SearchNode* sn = new SearchNode();
				sn->tile = _tileMap->getTile(i,j);
				//nodes[nodeSearch->tile] = sn;
				nodes.insert(std::make_pair(sn->tile, sn));
			}
		}

		for (size_t i = 0; i < _tileMap->getRowCount(); i++)
		{
			for (size_t j = 0; j < _tileMap->getColumnCount(); j++)
			{
				if (_tileMap->getTile(i, j)->getWeight() == 0)
					continue;
				SearchNode* sn = nodes[_tileMap->getTile(i,j)];
				//Need to get the row and colum of the current tile to set the adjacent tiles (neighbors)
				//sn->tile->getRow();
				//_tileMap->getTile(sn->tile->getRow() + 1, j);
				//check if they're nulls
				if (nodes[_tileMap->getTile(sn->tile->getRow() + 1, j)] != nullptr)
				sn->neighbors.push_back(nodes[_tileMap->getTile(sn->tile->getRow() + 1, j)]);
				if (nodes[_tileMap->getTile(sn->tile->getRow() - 1, j)] != nullptr)
				sn->neighbors.push_back(nodes[_tileMap->getTile(sn->tile->getRow() - 1, j)]);
				if (nodes[_tileMap->getTile(i, sn->tile->getColumn() + 1)] != nullptr)
				sn->neighbors.push_back(nodes[_tileMap->getTile(i, sn->tile->getColumn() + 1)]);
				if (nodes[_tileMap->getTile(i, sn->tile->getColumn() - 1)] != nullptr)
				sn->neighbors.push_back(nodes[_tileMap->getTile(i, sn->tile->getColumn() - 1)]);
				if (sn->tile->getRow() % 2 == 0)
				{
					// IS EVEN
					if (nodes[_tileMap->getTile(sn->tile->getRow() + 1, sn->tile->getColumn() - 1)] != nullptr)
					sn->neighbors.push_back(nodes[_tileMap->getTile(sn->tile->getRow() + 1, sn->tile->getColumn() - 1)]);
					if (nodes[_tileMap->getTile(sn->tile->getRow() - 1, sn->tile->getColumn() - 1)] != nullptr)
					sn->neighbors.push_back(nodes[_tileMap->getTile(sn->tile->getRow() - 1, sn->tile->getColumn() - 1)]);
				}
				else
				{
					// IS ODD
					if (nodes[_tileMap->getTile(sn->tile->getRow() + 1, sn->tile->getColumn() + 1)] != nullptr)
					sn->neighbors.push_back(nodes[_tileMap->getTile(sn->tile->getRow() + 1, sn->tile->getColumn() + 1)]);
					if (nodes[_tileMap->getTile(sn->tile->getRow() - 1, sn->tile->getColumn() + 1)] != nullptr)
					sn->neighbors.push_back(nodes[_tileMap->getTile(sn->tile->getRow() - 1, sn->tile->getColumn() + 1)]);
				}
			}
		}

	}

	void PathSearch::enter(int startRow, int startColumn, int goalRow, int goalColumn)
	{
		/*startRow = DEFAULT_START_ROW;
		startColumn = DEFAULT_START_COL;*/
		//goalRow = DEFAULT_GOAL_ROW;
		//goalColumn = DEFAULT_GOAL_COL;
		SearchNode* start = nodes[tileMap->getTile(startRow, startColumn)];
		goal = nodes[tileMap->getTile(goalRow, goalColumn)];
		PlannerNode* plannerStart = new PlannerNode();
		plannerStart->searchNode = start;
		plannerStart->parent = nullptr;
		plannerStart->givenCost = 0;
		plannerStart->heuristicCost = estimate(start, goal);
		plannerStart->finalCost = plannerStart->givenCost + plannerStart->heuristicCost * 1.2f;
		open.push(plannerStart);
		visited[start] = open.front();
	}

	float PathSearch::estimate(SearchNode* a, SearchNode* b)
	{
		return std::sqrt(pow((b->tile->getRow() - a->tile->getRow()), 2) + pow((b->tile->getColumn() - a->tile->getColumn()), 2));
	}

	void PathSearch::update(long timeslice)
	{
		while (!open.empty())
		{
			PlannerNode* current = open.front();
			open.pop();
			//add line to color hexes that have been passed even if not used for the path
			current->searchNode->tile->setFill(0xffff0000);
			if (current->searchNode->tile == goal->tile)
			{
				solutionList.clear();
				while (current != nullptr)
				{
					solutionList.push_back(current->searchNode->tile);
					if (current->parent != nullptr) 
					{
						current->searchNode->tile->setFill(0xffffff00);
						current->searchNode->tile->addLineTo(current->parent->searchNode->tile, 0xffff0000);
					}
					current = current->parent;
				}
				dom = true;
				return;
			}
			for (size_t i = 0; i < current->searchNode->neighbors.size(); i++)
			{
				SearchNode* successor = current->searchNode->neighbors[i];
				float tempGivenCost = current->givenCost + current->searchNode->neighbors[i]->tile->getWeight();
				if (visited[successor] != NULL)
				{
					PlannerNode* node = visited[successor];
					if (tempGivenCost < node->givenCost) {
						open.remove(node);
						node->parent = current;
						node->givenCost = tempGivenCost;
						node->finalCost = node->givenCost + node->heuristicCost * 1.2f;
						visited[successor] = node;
						open.push(node);
					}
				}
				else
				{
					PlannerNode* successorNode = new PlannerNode();
					successorNode->searchNode = successor;
					successorNode->parent = current;
					successorNode->givenCost = tempGivenCost;
					successorNode->heuristicCost = estimate(successor, goal);
					successorNode->finalCost = successorNode->givenCost + successorNode->heuristicCost * 1.2f;
					open.push(successorNode);
					visited[successor] = successorNode;
				}
			}
		}
	}

	void PathSearch::exit()
	{
		goal = nullptr;
		open.clear();
		visited.clear();
		dom = false;
	}

	void PathSearch::shutdown()
	{
		nodes.clear();
	}

	bool PathSearch::isDone() const
	{
		return dom;
	}

	std::vector<Tile const*> const PathSearch::getSolution() const
	{
		return solutionList;
	}
}}  // namespace fullsail_ai::algorithms

