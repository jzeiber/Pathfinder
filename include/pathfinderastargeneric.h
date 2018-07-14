#ifndef _pathfinder_astar_generic_
#define _pathfinder_astar_generic_

/*
	Pathfinder library is
	Copyright (C) 2008 Jeremy Zeiber
	
	Permission is given by the author to freely redistribute and 
	include this code in any program as long as this credit is 
	given where due.
	
	COVERED CODE IS PROVIDED UNDER THIS LICENSE ON AN "AS IS" BASIS, 
	WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, 
	INCLUDING, WITHOUT LIMITATION, WARRANTIES THAT THE COVERED CODE 
	IS FREE OF DEFECTS, MERCHANTABLE, FIT FOR A PARTICULAR PURPOSE
	OR NON-INFRINGING. THE ENTIRE RISK AS TO THE QUALITY AND 
	PERFORMANCE OF THE COVERED CODE IS WITH YOU. SHOULD ANY COVERED 
	CODE PROVE DEFECTIVE IN ANY RESPECT, YOU (NOT THE INITIAL 
	DEVELOPER OR ANY OTHER CONTRIBUTOR) ASSUME THE COST OF ANY 
	NECESSARY SERVICING, REPAIR OR CORRECTION. THIS DISCLAIMER OF 
	WARRANTY CONSTITUTES AN ESSENTIAL PART OF THIS LICENSE. NO USE 
	OF ANY COVERED CODE IS AUTHORIZED HEREUNDER EXCEPT UNDER
	THIS DISCLAIMER.
	 
	Use at your own risk!
*/

/**
	\file pathfinderastargeneric.h
*/

#include "pathfinder.h"

#include <functional>
#include <algorithm>
#include <map>

namespace Pathfinder
{

/**
	\brief Generic A * pathfinder
	
	This is a generic A * path finding class.  Use the cached version of this class if state change costs mostly remain the same.
	
	\tparam NODETYPE Defines a state of the pathfinder, typically an x,y coordinate for tile based games
	\tparam DESTCOST A functor implementing IDestinationCost
	\tparam GETNEIGHBORS A functor implementing IGetNeighbors
	\tparam COSTTYPE The cost type of moving from state to state, typically a float or double
*/
template <class NODETYPE, class DESTCOST, class GETNEIGHBORS, class COSTTYPE=double>
class AStarGeneric
{
public:
	AStarGeneric();
	~AStarGeneric();

	// find path from start to finish - blocking until path is found or not found
	/**
		\brief Finds a path between two states
		
		This method will block until a path from the start to the goal is found or all reachable nodes have been checked without finding a path.
		
		\param start The starting state
		\param goal The goal state
		\param[out] finalpath A vector of states from the starting state to the goal state
		\param destinationcost User supplied functor implementing IDestinationCost
		\param getneighbors User supplied functor implementing IGetNeighbors
		\return PATH_FOUND or PATH_NOTFOUND
	*/
	const PathResult FindPath(const NODETYPE &start, const NODETYPE &goal, std::vector<NODETYPE> &finalpath, DESTCOST &destinationcost, GETNEIGHBORS &getneighbors);

	// initialize finding path step by step
	/**
		\brief Initializes the path finder for single stepping
		
		\param start The starting state
		\param goal The goal state
		\param destcost User supplied functor IDestinationCost
		\param getneighbors User supplied functor IGetNeighbors
	*/
	void InitializeStep(const NODETYPE &start, const NODETYPE &goal, DESTCOST &destcost, GETNEIGHBORS &getneighbors);
	
	/**
		\brief Single steps the path finder
		
		\return PATH_FOUND, PATH_NOTFOUND, or PATH_SEARCHING if the user should call Step again to continue searching
	*/
	const PathResult Step();
	
	/**
		\brief Gets the current path
		
		This is usually called to get a complete path after Step returns PATH_FOUND, but it may be called at any step of the path finding, whether a complete path has been found or not.
		
		\param[out] path A vector of states from the starting state to the current state
	*/
	void GetPath(std::vector<NODETYPE> &path);

	/**
		\brief Functor passed to the IGetNeighbors functor
	*/
	class AddNeighborFunctor
	{
	public:
		AddNeighborFunctor(AStarGeneric &astar):m_astargeneric(astar)	{}
		/**
			\brief Adds a directly reachable neighbor state
			
			\param neighbor The neighbor state
			\param movecost The cost of moving from the parent state to this state
		*/
		inline void operator()(const NODETYPE &neighbor, const COSTTYPE &movecost)
		{
			m_astargeneric.AddNeighbor(neighbor,movecost);
		}
	private:
		AStarGeneric &m_astargeneric;
	};

	/**
		\brief Functor interface for getting the neighbor states of a given state
	*/
	class IGetNeighbors
	{
	public:
		virtual ~IGetNeighbors()=0;
		/**
			\brief Finds neighbors of a given state
			
			\param node The inital state we want the neighbors of
			\param addneighborfunctor An AddNeighborFunctor function object that must be called for every reachable neighbor of the parent state
		*/
		virtual void operator()(const NODETYPE &node, typename AStarGeneric<NODETYPE,DESTCOST,GETNEIGHBORS,COSTTYPE>::AddNeighborFunctor &addneighborfunctor)=0;
	};

private:
	// Can't be copied
	AStarGeneric(const AStarGeneric &rhs);
	AStarGeneric &operator=(const AStarGeneric &rhs);

	struct astarnode
	{
		NODETYPE m_node;
		astarnode *m_parent;
		COSTTYPE m_f;			// total cost to reach destination from source
		COSTTYPE m_g;			// actual cost from source to this node
		COSTTYPE m_h;			// estimated cost from this node to destination
	};

	struct astarnodecompare:public std::binary_function<astarnode *, astarnode *, bool>
	{
		inline bool operator()(const astarnode *left, const astarnode *right)
		{
			return right->m_f<left->m_f;
		}
	};

	class astarallocator:public std::allocator<astarnode>
	{
	public:
		astarallocator()
		{
			m_mempos=0;
			m_mem.resize(1000,NULL);
		}

		~astarallocator()
		{
			for(std::vector<astarnode *>::size_type i=0; i<m_mempos; i++)
			{
				delete m_mem[i];
			}
		}

		pointer allocate(size_type count)
		{
			return m_mempos>0 ? m_mem[--m_mempos] : new astarnode;
		}

		void deallocate(pointer ptr, size_type count)
		{
			if(m_mempos==m_mem.size())
			{
				m_mem.resize(m_mem.size()+1000,NULL);
			}
			m_mem[m_mempos++]=ptr;
		}

	private:
		std::vector<astarnode *> m_mem;
		typename std::vector<astarnode *>::size_type m_mempos;
	};

	struct listelement
	{
		listelement():m_node(NULL),m_onopen(false),m_onclosed(false)	{}
		astarnode *m_node;
		bool m_onopen;
		bool m_onclosed;
	};

	void ClearOpenList();
	void ClearClosedList();
	void AddNeighbor(const NODETYPE &neighbor, const COSTTYPE &movecost);

	astarallocator m_astarnodeallocator;			// allocates memory for astarnode

	std::vector<astarnode *> m_openlist;			// list of open nodes
	std::vector<astarnode *> m_closedlist;			// list of closed nodes - keep around just for the pointers
	std::map<NODETYPE, listelement> m_helperlist;	// helper list if node is on open/closed list

	astarnode *m_currentnode;
	astarnode *m_astaropen;
	const NODETYPE *m_start;
	const NODETYPE *m_goal;
	DESTCOST *m_destinationcost;
	GETNEIGHBORS *m_getneighbors;

};

template <class NODETYPE, class DESTCOST, class GETNEIGHBORS, class COSTTYPE>
AStarGeneric<NODETYPE,DESTCOST,GETNEIGHBORS,COSTTYPE>::AStarGeneric():
m_currentnode(0),m_astaropen(0),m_goal(0),m_destinationcost(0),m_getneighbors(0)
{

}

template <class NODETYPE, class DESTCOST, class GETNEIGHBORS, class COSTTYPE>
AStarGeneric<NODETYPE,DESTCOST,GETNEIGHBORS,COSTTYPE>::~AStarGeneric()
{
	ClearOpenList();
	ClearClosedList();
}

template <class NODETYPE, class DESTCOST, class GETNEIGHBORS, class COSTTYPE>
void AStarGeneric<NODETYPE,DESTCOST,GETNEIGHBORS,COSTTYPE>::AddNeighbor(const NODETYPE &neighbor, const COSTTYPE &movecost)
{
	// ignore node if it is already on the closed list
	if(m_helperlist[neighbor].m_onclosed==false)
	{

		// get the astarnode from the list - will be NULL if it doesn't already exist
		m_astaropen=m_helperlist[neighbor].m_node;

		// already on open list - recalculate values if F will be less using this path
		if(m_astaropen!=NULL)
		{
			if(m_currentnode->m_g+movecost<m_astaropen->m_g)
			{
				m_astaropen->m_g=m_currentnode->m_g+movecost;
				m_astaropen->m_f=m_astaropen->m_g+m_astaropen->m_h;
				m_astaropen->m_parent=m_currentnode;
				std::make_heap(m_openlist.begin(),m_openlist.end(),astarnodecompare());
			}
		}
		// not on the open list - calculate values and add
		else
		{
			astarnode *newnode=m_astarnodeallocator.allocate(1);
			newnode->m_node=neighbor;
			newnode->m_parent=m_currentnode;
			newnode->m_g=m_currentnode->m_g+movecost;
			newnode->m_h=m_destinationcost->operator()(neighbor,*m_goal);
			newnode->m_f=newnode->m_g+newnode->m_h;

			m_openlist.push_back(newnode);
			std::push_heap(m_openlist.begin(),m_openlist.end(),astarnodecompare());
			m_helperlist[newnode->m_node].m_onopen=true;
			m_helperlist[newnode->m_node].m_node=newnode;
		}

	}
}

template <class NODETYPE, class DESTCOST, class GETNEIGHBORS, class COSTTYPE>
void AStarGeneric<NODETYPE,DESTCOST,GETNEIGHBORS,COSTTYPE>::ClearClosedList()
{
	for(std::vector<astarnode *>::iterator i=m_closedlist.begin(); i!=m_closedlist.end(); i++)
	{
		m_astarnodeallocator.deallocate((*i),1);
	}
	m_closedlist.clear();
}

template <class NODETYPE, class DESTCOST, class GETNEIGHBORS, class COSTTYPE>
void AStarGeneric<NODETYPE,DESTCOST,GETNEIGHBORS,COSTTYPE>::ClearOpenList()
{
	for(std::vector<astarnode *>::iterator i=m_openlist.begin(); i!=m_openlist.end(); i++)
	{
		m_astarnodeallocator.deallocate((*i),1);
	}
	m_openlist.clear();
}

template <class NODETYPE, class DESTCOST, class GETNEIGHBORS, class COSTTYPE>
const PathResult AStarGeneric<NODETYPE,DESTCOST,GETNEIGHBORS,COSTTYPE>::FindPath(const NODETYPE &start, const NODETYPE &goal, std::vector<NODETYPE> &finalpath, DESTCOST &destinationcost, GETNEIGHBORS &getneighbors)
{
	// clear the lists
	ClearOpenList();
	ClearClosedList();
	m_helperlist.clear();

	m_astaropen=0;
	m_goal=&goal;
	m_destinationcost=&destinationcost;
	AddNeighborFunctor anf(*this);

	// setup current nodes
	m_currentnode=0;
	astarnode *newnode=m_astarnodeallocator.allocate(1);
	newnode->m_node=start;
	newnode->m_parent=NULL;
	newnode->m_g=destinationcost(start,goal);
	newnode->m_h=0;
	newnode->m_f=newnode->m_g+newnode->m_h;

	// push the start node on the open list
	m_openlist.push_back(newnode);
	std::push_heap(m_openlist.begin(),m_openlist.end(),astarnodecompare());
	m_helperlist[newnode->m_node].m_node=newnode;
	m_helperlist[newnode->m_node].m_onopen=true;

	while(m_openlist.size()>0)
	{
		m_currentnode=m_openlist.front();
		std::pop_heap(m_openlist.begin(),m_openlist.end(),astarnodecompare());	// front element has now been placed at the back of the vector
		m_openlist.pop_back();
		m_helperlist[m_currentnode->m_node].m_onopen=false;

		// push the current node on the closed list
		m_closedlist.push_back(m_currentnode);
		m_helperlist[m_currentnode->m_node].m_onclosed=true;

		if(m_currentnode->m_node!=goal)
		{
			getneighbors(m_currentnode->m_node,anf);
		}
		else
		{
			finalpath.clear();
			while(m_currentnode)
			{
				finalpath.push_back(m_currentnode->m_node);
				m_currentnode=m_currentnode->m_parent;
			}
			return PATH_FOUND;
		}

	}

	return PATH_NOTFOUND;
}

template <class NODETYPE, class DESTCOST, class GETNEIGHBORS, class COSTTYPE>
void AStarGeneric<NODETYPE,DESTCOST,GETNEIGHBORS,COSTTYPE>::GetPath(std::vector<NODETYPE> &path)
{
	path.clear();
	astarnode *cnode=m_currentnode;
	while(cnode)
	{
		path.push_back(cnode->m_node);
		cnode=cnode->m_parent;
	}
}

template <class NODETYPE, class DESTCOST, class GETNEIGHBORS, class COSTTYPE>
void AStarGeneric<NODETYPE,DESTCOST,GETNEIGHBORS,COSTTYPE>::InitializeStep(const NODETYPE &start, const NODETYPE &goal, DESTCOST &destinationcost, GETNEIGHBORS &getneighbors)
{
	m_start=&start;
	m_goal=&goal;
	m_destinationcost=&destinationcost;
	m_getneighbors=&getneighbors;

	// clear the lists
	ClearOpenList();
	ClearClosedList();
	m_helperlist.clear();

	m_astaropen=0;
	m_currentnode=0;

	astarnode *newnode=m_astarnodeallocator.allocate(1);
	newnode->m_node=start;
	newnode->m_parent=NULL;
	newnode->m_g=destinationcost(start,goal);
	newnode->m_h=0;
	newnode->m_f=newnode->m_g+newnode->m_h;

	// push the start node on the open list
	m_openlist.push_back(newnode);
	std::push_heap(m_openlist.begin(),m_openlist.end(),astarnodecompare());
	m_helperlist[newnode->m_node].m_node=newnode;
	m_helperlist[newnode->m_node].m_onopen=true;

}

template <class NODETYPE, class DESTCOST, class GETNEIGHBORS, class COSTTYPE>
const PathResult AStarGeneric<NODETYPE,DESTCOST,GETNEIGHBORS,COSTTYPE>::Step()
{
	if(m_openlist.size()>0)
	{
		m_currentnode=m_openlist.front();
		std::pop_heap(m_openlist.begin(),m_openlist.end(),astarnodecompare());	// front element has now been placed at the back of the vector
		m_openlist.pop_back();
		m_helperlist[m_currentnode->m_node].m_onopen=false;

		// push the current node on the closed list
		m_closedlist.push_back(m_currentnode);
		m_helperlist[m_currentnode->m_node].m_onclosed=true;

		if(m_currentnode->m_node!=*m_goal)
		{
			m_getneighbors->operator()(m_currentnode->m_node,AddNeighborFunctor(*this));
		}
		else
		{
			return PATH_FOUND;
		}

		return PATH_SEARCHING;

	}

	return PATH_NOTFOUND;
}

}	// namespace

#endif	// _pathfinder_astar_generic_
