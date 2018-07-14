#include <pathfinderastargeneric.h>

#include <vector>
#include <ctime>
#include <iostream>

const long mapwidth=80;
const long mapheight=25;

class DestinationCost
{
public:
	const double operator()(const std::pair<long,long> &node, const std::pair<long,long> &goal)
	{
		return abs(node.first-goal.first)+abs(node.second-goal.second);	
	}
};

class GetNeighbors
{
public:
	GetNeighbors(std::vector<char> &themap):m_map(themap)
	{
	}

	inline void operator()(const std::pair<long,long> &node, Pathfinder::AStarGeneric<std::pair<long,long>,DestinationCost,GetNeighbors>::AddNeighborFunctor &an)
	{
		for(int y=node.second-1; y<=node.second+1; y++)
		{
			for(int x=node.first-1; x<=node.first+1; x++)
			{
				if((x!=node.first || y!=node.second) && y>=0 && y<mapheight && x>=0 && x<mapwidth && m_map[(y*mapwidth)+x]!='#')
				{
					an(std::pair<long,long>(x,y),(node.first!=x && node.second!=y) ? 1.4 : 1.0);
				}
			}
		}
	}
private:
	std::vector<char> &m_map;
};

void loadmap(const std::string &filename, std::vector<char> &themap)
{
	themap.clear();
	themap.resize(mapwidth*mapheight,0);

	FILE *infile=fopen(filename.c_str(),"rb");
	if(infile)
	{
		fseek(infile,0,SEEK_END);
		long filelen=ftell(infile);
		fseek(infile,0,SEEK_SET);

		std::vector<char> data(filelen,0);
		fread(&data[0],1,data.size(),infile);

		std::vector<char>::size_type pos=0;
		for(int y=0; y<mapheight; y++)
		{
			for(int x=0; x<mapwidth; x++)
			{
				themap[(y*mapwidth)+x]=data[pos++];
			}
			pos++;
		}

		fclose(infile);
	}
}

void showmap(const std::vector<char> &themap)
{
	for(int y=0; y<mapheight; y++)
	{
		for(int x=0; x<mapwidth; x++)
		{
			std::cout << themap[(y*mapwidth)+x];
		}
	}
}

void showmappath(const std::vector<char> &themap, const std::vector<std::pair<long,long> > &path)
{
	std::vector<char> mappath(themap);

	for(std::vector<std::pair<long,long> >::const_iterator i=path.begin(); i!=path.end(); i++)
	{
		mappath[((*i).second*mapwidth)+(*i).first]='*';
	}

	for(int y=0; y<mapheight; y++)
	{
		for(int x=0; x<mapwidth; x++)
		{
			std::cout << mappath[(y*mapwidth)+x];
		}
	}

}

int main()
{

	srand(time(0));

	std::pair<long,long> startpos((rand()%(mapwidth-2))+1,(rand()%(mapheight-2))+1);
	std::pair<long,long> endpos((rand()%(mapwidth-2))+1,(rand()%(mapheight-2))+1);
	std::vector<char> themap;
	std::vector<std::pair<long,long> > finalpath;
	
	Pathfinder::AStarGeneric<std::pair<long,long>,DestinationCost,GetNeighbors,double> astargeneric;
	DestinationCost dc;
	GetNeighbors gn(themap);

	loadmap("map.txt",themap);

	int rval=astargeneric.FindPath(startpos,endpos,finalpath,dc,gn);
	
	switch(rval)
	{
	case Pathfinder::PATH_FOUND:
		std::cout << "Found path ";
		break;
	case Pathfinder::PATH_NOTFOUND:
		std::cout << "Could not find path ";
	}
	std::cout << "(" << startpos.first << "," << startpos.second << ") to (" << endpos.first << "," << endpos.second << ")" << std::endl;
	showmappath(themap,finalpath);
	
	return 0;
}
