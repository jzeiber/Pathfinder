#ifndef _pathfinder_astar_tile8dir_cached_
#define _pathfinder_astar_tile8dir_cached_

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
	\file pathfinderastartile8dircached.h
*/

#include "pathfinder.h"

#include <functional>
#include <algorithm>
#include <map>

namespace Pathfinder
{

/**
	\brief A * path finder for 2d movement in 8 directions with caching
	
	This is an A * path finder for tile based games where movement is possible in 8 directions surrounding each tile.    The movement costs from state to state are cached to maximize performance.  If state change costs can change frequently, you should use the non-cached version of this class.
	
	\tparam MOVECOST A functor implementing IMoveCost
	\tparam DESTCOST A functor implementing IDestinationCost
	\tparam MOVEBLOCKED A functor implementing IMoveBlocked
	\tparam COSTTYPE The cost type of moving from state to state, typically a float or double
*/
template <class MOVECOST, class DESTCOST, class MOVEBLOCKED, class COSTTYPE=double>
class AStarTile8DirCached
{
public:
	AStarTile8DirCached();
	~AStarTile8DirCached();

	/**
		\brief Finds a path from one x,y position to another
		
		This method finds a path from one state to another and assumes that all 8 surrounding tiles are always reachable.
		This method will block until a path from the start to the goal is found or all reachable nodes have been checked without finding a path.
		
		\param start The starting x,y position
		\param goal The goal x,y position
		\param[out] finalpath A vector of x,y positions from the starting position to the goal position
		\param movecost A functor implementing IMoveCost
		\param destinationcost A functor implementing IDestinationCost
		\return PATH_FOUND or PATH_NOTFOUND
	*/
	const PathResult FindPath(const std::pair<long,long> &start, const std::pair<long,long> &goal, std::vector<std::pair<long,long> > &finalpath, MOVECOST &movecost, DESTCOST &destinationcost);
	/**
		\brief Finds a path from one x,y position to another
		
		This method finds a path from one state to another and takes into consideration that any of the surrounding tiles may not be reachable.
		This method will block until a path from the start to the goal is found or all reachable nodes have been checked without finding a path.
		
		\param start The starting x,y position
		\param goal The goal x,y position
		\param[out] finalpath A vector of x,y positions from the starting position to the goal position
		\param movecost A functor implementing IMoveCost
		\param moveblocked A functor implementing IMoveBlocked
		\param destinationcost A functor implementing IDestinationCost
		\return PATH_FOUND or PATH_NOTFOUND		
	*/
	const PathResult FindPath(const std::pair<long,long> &start, const std::pair<long,long> &goal, std::vector<std::pair<long,long> > &finalpath, MOVECOST &movecost, MOVEBLOCKED &moveblocked, DESTCOST &destinationcost);

	/**
		\brief Sets the map width
		
		You must call this method with the width of the map before performing any path finding
		
		\param width The width of the map
	*/
	void SetMapWidth(const long width);
	
	/**
		\brief Clears the entire cache
	*/
	void ClearCache();
	
	/**
		\brief Clears one cache element
		
		Call this method when the cost of reaching a state has changed.
		
		\param pos The position of the cache to clear
	*/
	void ClearCachePosition(const std::pair<long,long> &pos);

private:
	// Can't be copied
	AStarTile8DirCached(const AStarTile8DirCached &rhs);
	AStarTile8DirCached &operator=(const AStarTile8DirCached &rhs);

	struct astarnode
	{
		std::pair<long,long> m_node;
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

	astarallocator m_astarnodeallocator;			// allocates memory for astarnode

	std::vector<astarnode *> m_openlist;			// list of open nodes
	std::vector<astarnode *> m_closedlist;			// list of closed nodes - keep around just for the pointers
	std::vector<listelement> m_helperlist;
	long m_helperliststartindex;
	
	long m_mapwidth;

	struct cacheelement
	{
		cacheelement():m_hascache(false)	{}
		bool m_hascache;
		std::vector<std::pair<std::pair<long,long> ,COSTTYPE> > m_neighbors;
	};
	std::vector<cacheelement> m_cache;
	long m_cachestartindex;

	inline listelement &hl(const long index)
	{
		if(m_helperlist.size()==0)
		{
			m_helperlist.push_back(listelement());
			m_helperliststartindex=index;
		}
		else if(index<m_helperliststartindex)
		{
			m_helperlist.insert(m_helperlist.begin(),m_helperliststartindex-index,listelement());
			m_helperliststartindex=index;
		}
		else if(index-m_helperliststartindex>=m_helperlist.size())
		{
			m_helperlist.insert(m_helperlist.end(),((index-m_helperliststartindex)+1)-m_helperlist.size(),listelement());
		}

		return m_helperlist[index-m_helperliststartindex];
	}

};

template <class MOVECOST, class DESTCOST, class MOVEBLOCKED, class COSTTYPE>
AStarTile8DirCached<MOVECOST,DESTCOST,MOVEBLOCKED,COSTTYPE>::AStarTile8DirCached():m_mapwidth(0),m_cachestartindex(0),m_helperliststartindex(0)
{

}

template <class MOVECOST, class DESTCOST, class MOVEBLOCKED, class COSTTYPE>
AStarTile8DirCached<MOVECOST,DESTCOST,MOVEBLOCKED,COSTTYPE>::~AStarTile8DirCached()
{
	ClearOpenList();
	ClearClosedList();
}

template <class MOVECOST, class DESTCOST, class MOVEBLOCKED, class COSTTYPE>
void AStarTile8DirCached<MOVECOST,DESTCOST,MOVEBLOCKED,COSTTYPE>::ClearCache()
{
	m_cache.clear();
	m_cachestartindex=0;	
}

template <class MOVECOST, class DESTCOST, class MOVEBLOCKED, class COSTTYPE>
void AStarTile8DirCached<MOVECOST,DESTCOST,MOVEBLOCKED,COSTTYPE>::ClearCachePosition(const std::pair<long,long> &pos)
{
	long index=(pos.second*m_mapwidth)+pos.first;
	if(index>=m_cachestartindex && index-m_cachestartindex<m_cache.size())
	{
		m_cache[index-m_cachestartindex].m_hascache=false;
		m_cache[index-m_cachestartindex].m_neighbors.clear();
	}
}

template <class MOVECOST, class DESTCOST, class MOVEBLOCKED, class COSTTYPE>
void AStarTile8DirCached<MOVECOST,DESTCOST,MOVEBLOCKED,COSTTYPE>::ClearClosedList()
{
	for(std::vector<astarnode *>::iterator i=m_closedlist.begin(); i!=m_closedlist.end(); i++)
	{
		m_astarnodeallocator.deallocate((*i),1);
	}
	m_closedlist.clear();
}

template <class MOVECOST, class DESTCOST, class MOVEBLOCKED, class COSTTYPE>
void AStarTile8DirCached<MOVECOST,DESTCOST,MOVEBLOCKED,COSTTYPE>::ClearOpenList()
{
	for(std::vector<astarnode *>::iterator i=m_openlist.begin(); i!=m_openlist.end(); i++)
	{
		m_astarnodeallocator.deallocate((*i),1);
	}
	m_openlist.clear();
}

template <class MOVECOST, class DESTCOST, class MOVEBLOCKED, class COSTTYPE>
const PathResult AStarTile8DirCached<MOVECOST,DESTCOST,MOVEBLOCKED,COSTTYPE>::FindPath(const std::pair<long,long> &start, const std::pair<long,long> &goal, std::vector<std::pair<long,long> > &finalpath, MOVECOST &movecost, DESTCOST &destinationcost)
{
	// clear the lists
	ClearOpenList();
	ClearClosedList();
	m_helperlist.clear();
	m_helperliststartindex=0;

	long xx;
	long yy;

	astarnode *astaropen;

	// setup current nodes
	astarnode *currentnode;
	astarnode *newnode=m_astarnodeallocator.allocate(1);
	newnode->m_node=start;
	newnode->m_parent=NULL;
	newnode->m_g=destinationcost(start,goal);
	newnode->m_h=0;
	newnode->m_f=newnode->m_g+newnode->m_h;

	// push the start node on the open list
	m_openlist.push_back(newnode);
	std::push_heap(m_openlist.begin(),m_openlist.end(),astarnodecompare());
	hl((newnode->m_node.second*m_mapwidth)+newnode->m_node.first).m_node=newnode;
	m_helperlist[((newnode->m_node.second*m_mapwidth)+newnode->m_node.first)-m_helperliststartindex].m_onopen=true;

	while(m_openlist.size()>0)
	{
		currentnode=m_openlist.front();
		std::pop_heap(m_openlist.begin(),m_openlist.end(),astarnodecompare());	// front element has now been placed at the back of the vector
		m_openlist.pop_back();
		m_helperlist[((currentnode->m_node.second*m_mapwidth)+currentnode->m_node.first)-m_helperliststartindex].m_onopen=false;

		// push the current node on the closed list
		m_closedlist.push_back(currentnode);
		m_helperlist[((currentnode->m_node.second*m_mapwidth)+currentnode->m_node.first)-m_helperliststartindex].m_onclosed=true;

		if(currentnode->m_node!=goal)
		{
			long currentnodeindex=((currentnode->m_node.second*m_mapwidth)+currentnode->m_node.first);
			if(currentnodeindex>=m_cachestartindex && currentnodeindex<m_cachestartindex+m_cache.size() && m_cache[currentnodeindex-m_cachestartindex].m_hascache)
			{
				// add neighbors from cache
				for(std::vector<std::pair<std::pair<long,long> ,COSTTYPE> >::iterator i=m_cache[currentnodeindex-m_cachestartindex].m_neighbors.begin(); i!=m_cache[currentnodeindex-m_cachestartindex].m_neighbors.end(); i++)
				{
					xx=(*i).first.first;
					yy=(*i).first.second;
					if(hl((yy*m_mapwidth)+xx).m_onclosed==false)
					{

						// get the astarnode from the list - will be NULL if it doesn't already exist
						astaropen=m_helperlist[((yy*m_mapwidth)+xx)-m_helperliststartindex].m_node;

						// already on open list - recalculate values if F will be less using this path
						if(astaropen!=NULL)
						{
							if(currentnode->m_g+movecost(currentnode->m_node,std::pair<long,long>(xx,yy))<astaropen->m_g)
							{
								astaropen->m_g=currentnode->m_g+movecost(currentnode->m_node,std::pair<long,long>(xx,yy));
								astaropen->m_f=astaropen->m_g+astaropen->m_h;
								astaropen->m_parent=currentnode;
								std::make_heap(m_openlist.begin(),m_openlist.end(),astarnodecompare());
							}
						}
						// not on the open list - calculate values and add
						else
						{
							newnode=m_astarnodeallocator.allocate(1);
							newnode->m_node=std::pair<long,long>(xx,yy);
							newnode->m_parent=currentnode;
							newnode->m_g=currentnode->m_g+movecost(currentnode->m_node,std::pair<long,long>(xx,yy));
							newnode->m_h=destinationcost(std::pair<long,long>(xx,yy),goal);
							newnode->m_f=newnode->m_g+newnode->m_h;

							m_openlist.push_back(newnode);
							std::push_heap(m_openlist.begin(),m_openlist.end(),astarnodecompare());
							hl((yy*m_mapwidth)+xx).m_onopen=true;
							m_helperlist[((yy*m_mapwidth)+xx)-m_helperliststartindex].m_node=newnode;
						}
					}
				}
			}
			// get neighbors and add to cache
			else
			{

				if(m_cache.size()==0)
				{
					m_cache.push_back(cacheelement());
					m_cachestartindex=currentnodeindex;
				}
				else if(currentnodeindex<m_cachestartindex)
				{
					m_cache.insert(m_cache.begin(),m_cachestartindex-currentnodeindex,cacheelement());
					m_cachestartindex=currentnodeindex;
				}
				else if(currentnodeindex-m_cachestartindex>=m_cache.size())
				{
					m_cache.insert(m_cache.end(),((currentnodeindex-m_cachestartindex)+1)-m_cache.size(),cacheelement());
				}
				m_cache[currentnodeindex-m_cachestartindex].m_hascache=true;

				for(yy=currentnode->m_node.second-1; yy<=currentnode->m_node.second+1; yy++)
				{
					for(xx=currentnode->m_node.first-1; xx<=currentnode->m_node.first+1; xx++)
					{
						if(xx!=currentnode->m_node.first || yy!=currentnode->m_node.second)
						{
							
							// ignore node if it is already on the closed list
							if(hl((yy*m_mapwidth)+xx).m_onclosed==false)
							{
	
								// get the astarnode from the list - will be NULL if it doesn't already exist
								astaropen=m_helperlist[((yy*m_mapwidth)+xx)-m_helperliststartindex].m_node;
	
								// already on open list - recalculate values if F will be less using this path
								if(astaropen!=NULL)
								{
									if(currentnode->m_g+movecost(currentnode->m_node,std::pair<long,long>(xx,yy))<astaropen->m_g)
									{
										astaropen->m_g=currentnode->m_g+movecost(currentnode->m_node,std::pair<long,long>(xx,yy));
										astaropen->m_f=astaropen->m_g+astaropen->m_h;
										astaropen->m_parent=currentnode;
										std::make_heap(m_openlist.begin(),m_openlist.end(),astarnodecompare());
									}
								}
								// not on the open list - calculate values and add
								else
								{
									newnode=m_astarnodeallocator.allocate(1);
									newnode->m_node=std::pair<long,long>(xx,yy);
									newnode->m_parent=currentnode;
									newnode->m_g=currentnode->m_g+movecost(currentnode->m_node,std::pair<long,long>(xx,yy));
									newnode->m_h=destinationcost(std::pair<long,long>(xx,yy),goal);
									newnode->m_f=newnode->m_g+newnode->m_h;
	
									m_openlist.push_back(newnode);
									std::push_heap(m_openlist.begin(),m_openlist.end(),astarnodecompare());
									hl((yy*m_mapwidth)+xx).m_onopen=true;
									m_helperlist[((yy*m_mapwidth)+xx)-m_helperliststartindex].m_node=newnode;
								}

								m_cache[currentnodeindex-m_cachestartindex].m_neighbors.push_back(std::pair<std::pair<long,long> ,COSTTYPE>(std::pair<long,long>(xx,yy),movecost(currentnode->m_node,std::pair<long,long>(xx,yy))));

							}
						}
					}
				}
			}
		}
		else
		{
			finalpath.clear();
			while(currentnode)
			{
				finalpath.push_back(currentnode->m_node);
				currentnode=currentnode->m_parent;
			}
			return PATH_FOUND;
		}

	}

	return PATH_NOTFOUND;
}

template <class MOVECOST, class DESTCOST, class MOVEBLOCKED, class COSTTYPE>
const PathResult AStarTile8DirCached<MOVECOST,DESTCOST,MOVEBLOCKED,COSTTYPE>::FindPath(const std::pair<long,long> &start, const std::pair<long,long> &goal, std::vector<std::pair<long,long> > &finalpath, MOVECOST &movecost, MOVEBLOCKED &moveblocked, DESTCOST &destinationcost)
{
	// clear the lists
	ClearOpenList();
	ClearClosedList();
	m_helperlist.clear();
	m_helperliststartindex=0;

	long xx;
	long yy;

	astarnode *astaropen;

	// setup current nodes
	astarnode *currentnode;
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
		currentnode=m_openlist.front();
		std::pop_heap(m_openlist.begin(),m_openlist.end(),astarnodecompare());	// front element has now been placed at the back of the vector
		m_openlist.pop_back();
		m_helperlist[currentnode->m_node].m_onopen=false;

		// push the current node on the closed list
		m_closedlist.push_back(currentnode);
		m_helperlist[currentnode->m_node].m_onclosed=true;

		if(currentnode->m_node!=goal)
		{

			long currentnodeindex=((currentnode->m_node.second*m_mapwidth)+currentnode->m_node.first);
			if(currentnodeindex>=m_cachestartindex && currentnodeindex<m_cachestartindex+m_cache.size() && m_cache[currentnodeindex-m_cachestartindex].m_hascache)
			{
				// add neighbors from cache
				for(std::vector<std::pair<std::pair<long,long> ,COSTTYPE> >::iterator i=m_cache[currentnodeindex-m_cachestartindex].m_neighbors.begin(); i!=m_cache[currentnodeindex-m_cachestartindex].m_neighbors.end(); i++)
				{
					xx=(*i).first.first;
					yy=(*i).first.second;
					if(hl((yy*m_mapwidth)+xx).m_onclosed==false)
					{

						// get the astarnode from the list - will be NULL if it doesn't already exist
						astaropen=m_helperlist[((yy*m_mapwidth)+xx)-m_helperliststartindex].m_node;

						// already on open list - recalculate values if F will be less using this path
						if(astaropen!=NULL)
						{
							if(currentnode->m_g+movecost(currentnode->m_node,std::pair<long,long>(xx,yy))<astaropen->m_g)
							{
								astaropen->m_g=currentnode->m_g+movecost(currentnode->m_node,std::pair<long,long>(xx,yy));
								astaropen->m_f=astaropen->m_g+astaropen->m_h;
								astaropen->m_parent=currentnode;
								std::make_heap(m_openlist.begin(),m_openlist.end(),astarnodecompare());
							}
						}
						// not on the open list - calculate values and add
						else
						{
							newnode=m_astarnodeallocator.allocate(1);
							newnode->m_node=std::pair<long,long>(xx,yy);
							newnode->m_parent=currentnode;
							newnode->m_g=currentnode->m_g+movecost(currentnode->m_node,std::pair<long,long>(xx,yy));
							newnode->m_h=destinationcost(std::pair<long,long>(xx,yy),goal);
							newnode->m_f=newnode->m_g+newnode->m_h;

							m_openlist.push_back(newnode);
							std::push_heap(m_openlist.begin(),m_openlist.end(),astarnodecompare());
							hl((yy*m_mapwidth)+xx).m_onopen=true;
							m_helperlist[((yy*m_mapwidth)+xx)-m_helperliststartindex].m_node=newnode;
						}
					}
				}
			}
			// get neighbors and add to cache
			else
			{

				if(m_cache.size()==0)
				{
					m_cache.push_back(cacheelement());
					m_cachestartindex=currentnodeindex;
				}
				else if(currentnodeindex<m_cachestartindex)
				{
					m_cache.insert(m_cache.begin(),m_cachestartindex-currentnodeindex,cacheelement());
					m_cachestartindex=currentnodeindex;
				}
				else if(currentnodeindex-m_cachestartindex>=m_cache.size())
				{
					m_cache.insert(m_cache.end(),((currentnodeindex-m_cachestartindex)+1)-m_cache.size(),cacheelement());
				}

				for(yy=currentnode->m_node.second-1; yy<=currentnode->m_node.second+1; yy++)
				{
					for(xx=currentnode->m_node.first-1; xx<=currentnode->m_node.first+1; xx++)
					{
						if(xx!=currentnode->m_node.first || yy!=currentnode->m_node.second)
						{
							
							// ignore node if it is already on the closed list
							if(moveblocked(currentnode->m_node,std::pair<long,long>(xx,yy))==false && hl((yy*m_mapwidth)+xx).m_onclosed==false)
							{
	
								// get the astarnode from the list - will be NULL if it doesn't already exist
								astaropen=m_helperlist[((yy*m_mapwidth)+xx)-m_helperliststartindex].m_node;
	
								// already on open list - recalculate values if F will be less using this path
								if(astaropen!=NULL)
								{
									if(currentnode->m_g+movecost(currentnode->m_node,std::pair<long,long>(xx,yy))<astaropen->m_g)
									{
										astaropen->m_g=currentnode->m_g+movecost(currentnode->m_node,std::pair<long,long>(xx,yy));
										astaropen->m_f=astaropen->m_g+astaropen->m_h;
										astaropen->m_parent=currentnode;
										std::make_heap(m_openlist.begin(),m_openlist.end(),astarnodecompare());
									}
								}
								// not on the open list - calculate values and add
								else
								{
									newnode=m_astarnodeallocator.allocate(1);
									newnode->m_node=std::pair<long,long>(xx,yy);
									newnode->m_parent=currentnode;
									newnode->m_g=currentnode->m_g+movecost(currentnode->m_node,std::pair<long,long>(xx,yy));
									newnode->m_h=destinationcost(std::pair<long,long>(xx,yy),goal);
									newnode->m_f=newnode->m_g+newnode->m_h;
	
									m_openlist.push_back(newnode);
									std::push_heap(m_openlist.begin(),m_openlist.end(),astarnodecompare());
									hl((yy*m_mapwidth)+xx).m_onopen=true;
									m_helperlist[((yy*m_mapwidth)+xx)-m_helperliststartindex].m_node=newnode;
								}

								m_cache[currentnodeindex-m_cachestartindex].m_hascache=true;
								m_cache[currentnodeindex-m_cachestartindex].m_neighbors.push_back(std::pair<std::pair<long,long> ,COSTTYPE>(std::pair<long,long>(xx,yy),movecost(currentnode->m_node,std::pair<long,long>(xx,yy))));

							}
						}
					}
				}
			}
		}
		else
		{
			finalpath.clear();
			while(currentnode)
			{
				finalpath.push_back(currentnode->m_node);
				currentnode=currentnode->m_parent;
			}
			return PATH_FOUND;
		}

	}

	return PATH_NOTFOUND;
}

template <class MOVECOST, class DESTCOST, class MOVEBLOCKED, class COSTTYPE>
void AStarTile8DirCached<MOVECOST,DESTCOST,MOVEBLOCKED,COSTTYPE>::SetMapWidth(const long width)
{
	if(width!=m_mapwidth)
	{
		m_mapwidth=width;
		ClearCache();
	}
}

}	// namespace

#endif	// _pathfinder_astar_tile8dir_
