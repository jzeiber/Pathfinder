#ifndef _pathfinder_
#define _pathfinder_

/**
	\mainpage Pathfinder library
	
	<b>version 1.0.0</b>
	
	The pathfinder library is comprised of several C++ template classes to facilitate finding connected paths from one state to another. 
	It makes use of STL data structures and algorithms to provide maximum performance.

	%Pathfinder library is
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
	\file pathfinder.h
	
	Main header file
*/

#include <vector>

/**
	\brief %Pathfinder namespace
	
	This namespace contains the %Pathfinder library
*/
namespace Pathfinder
{

/**
	\brief return values of path finding methods
*/
typedef enum PathResult
{
	/**
		A path was found from the source to the destination
	*/
	PATH_FOUND=1,
	/**
		There is no path from the source to the destination
	*/
	PATH_NOTFOUND=2,
	/**
		The path is still being constructed
	*/
	PATH_SEARCHING=3
};

/**
	\brief Functor interface to compute exact movement cost from one state to another
	
	\tparam NODETYPE Defines a state of the pathfinder, typically an x,y coordinate for tile based games
	\tparam COSTTYPE The cost type of moving from state to state, typically a float or double
*/
template <class NODETYPE, class COSTTYPE=double>
class IMoveCost
{
public:
	virtual ~IMoveCost()			{}
	/**
		\brief Computes movement cost from start to end
		
		This method computes the exact movement cost from one state to another.  
		
		\param start The initial state
		\param end The final state
		\return The exact cost of changing the state from start to end
	*/
	virtual const COSTTYPE operator()(const NODETYPE &start, const NODETYPE &end)=0;
};

/**
	\brief Functor interface to compute estimated movement cost from one state to the goal state
	
	\tparam NODETYPE Defines a state of the pathfinder, typically an x,y coordinate for tile based games
	\tparam COSTTYPE The cost type of moving from state to state, typically a float or double
*/
template <class NODETYPE, class COSTTYPE=double>
class IDestinationCost
{
public:
	virtual ~IDestinationCost()		{}
	/**
		\brief Computes estimated movement cost from node to goal
		
		This method computes an estimated movement cost from one state to another.
		
		\param node The starting state
		\param goal The final state
		\return The estimated cost of changing the state from node to goal
	*/
	virtual const COSTTYPE operator()(const NODETYPE &node, const NODETYPE &goal)=0;
};

/**
	\brief Functor interface to determine if direct state change is possible
*/
template <class NODETYPE>
class IMoveBlocked
{
public:
	virtual ~IMoveBlocked()			{}
	/**
		\brief Determines if a direct state change is possible
		
		For this method to return true, the path from start to end must be direct, with no intermediate states between.
		
		\param start The initial state
		\param end The final state
		\return true end is directly reachable from start, false otherwise
	*/
	virtual const bool operator()(const NODETYPE &start, const NODETYPE &end)=0;	
};

}	// namespace

#endif	// _pathfinder_
