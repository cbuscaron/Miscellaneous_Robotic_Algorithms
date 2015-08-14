#include <vector>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <map>
#include <iostream>
#include <utility>
#include <memory>


/**
 * Represent 2D posistion on grid
 */
struct Position
{
  Position() : x(0), y(0) { }
  Position(int x, int y) : x(x), y(y) { }
  int x,y;
};


/**
 * 2d grid, backed by std::vector
 */
template<typename T>
class Grid
{
public:
  Grid(int x_size, int y_size, T value) :
    x_size_(x_size), y_size_(y_size), data_(x_size*y_size, value)
  {

  }

  /**
   * Gets values at x,y
   */
  const T& get(int x, int y) const
  {
    check(x,y);
    return data_[y*x_size_ + x];
  }

  /**
   * Sets value at x,y
   */
  void set(int x, int y, const T& value)
  {
    check(x,y);
    data_[y*x_size_ + x] = value;
  }

  /**
   * Checks value at x,y to make sure it is in the grid.
   * Throws exception is if it not
   */
  void check(int x, int y) const
  {
    if ((x>=x_size_) || (x<0) || (y>=y_size_) || (y<0))
    {
      std::stringstream ss;
      ss << "position (" << x << "," << y << ")"
         << "outside of bounds (" << x_size_ << "," << y_size_ << ")" << std::endl;
      throw std::runtime_error(ss.str());
    }
  }

  /**
   * Print grid data to output stream
   */
  friend std::ostream& operator<<(std::ostream& os, const Grid<T>& grid)
  {
    for (int y=0; y<grid.y_size_; ++y)
    {
      for (int x=0; x<grid.x_size_; ++x)
      {
        os << grid.get(x,y);
      }
      os << std::endl;
    }
    return os;
  }

  inline int getXSize() const { return x_size_; }
  inline int getYSize() const { return y_size_; }

protected:
  int x_size_, y_size_;
  std::vector<T> data_;
};


/** Map from movement characters (U, D, R, L) to position delta's */
typedef std::map<char, Position> MovementMap;
MovementMap movements;

/** Map from tile types ('#', '.', 'S' ...) to cost of moving onto that tile */
std::map<char, int> costs;


/**
 * Finds and returns position of start tile 'S'
 */
Position findStart(const Grid<char> &maze)
{
  for (int x=0; x<maze.getXSize(); ++x)
  {
    for (int y=0; y<maze.getYSize(); ++y)
    {
      if (maze.get(x,y) == 'S')
      {
        return Position(x,y);
      }
    }
  }
  throw std::runtime_error("Could not find start");
}


/**
 * Loads maze from input stream and returns Grid object
 * Assumes input data has correct formatting.  Does a small amount of error checking.
 */
std::auto_ptr<Grid<char> > loadMaze(std::istream &inp)
{
  int size_x, size_y;
  inp >> size_x >> size_y;
  inp.ignore(1,'\n');
  std::auto_ptr<Grid<char> > maze(new Grid<char>(size_x, size_y, ' '));
  int y=0;
  while (inp.good())
  {
    std::string line;
    std::getline(inp, line);
    if (line.size() == 0)
    {
      break;
    }
    if (y>=size_y)
    {
      throw std::runtime_error("Too many lines of input");
    }
    if (static_cast<int>(line.size()) != size_x)
    {
      std::stringstream ss;
      ss << "Line " << y << " of map has wrong number of characters."
         << " Got " << line.size() << " expected " << size_x;
      throw std::runtime_error(ss.str());
    }
    for (int x=0; x<size_x; ++x)
    {
      maze->set(x,y,line[x]);
    }
    ++y;
  }
  if (y != size_y)
  {
    std::stringstream ss;
    ss << "Not enough lines of input"
       << " Got " << y
       << " Expected " << size_y
       << std::endl;
    throw std::runtime_error(ss.str());
  }
  return maze;
}


/**
 * Shows path on map by marking it with '*' on each step of path,
 * and then prints updated map to std::cout
 */
void plotPath(const Grid<char> &_maze, const std::string &path)
{
  Grid<char> maze(_maze); // copy maze, then modify copy
  Position position = findStart(maze);
  for (size_t ii = 0; ii<path.size();  ++ii)
  {
    Position delta = movements.at(path[ii]);
    position.x += delta.x;
    position.y += delta.y;
    maze.set(position.x, position.y, '*');
  }
  std::cout << maze << std::endl;
}


struct Node
{
  Node(int cost, const Position &position, const std::string &path) :
    cost(cost),
    position(position),
    path(path)
  {
    //empty
  }
  int cost;
  Position position;
  std::string path;
};

struct NodeCostIsLower
{
  bool operator()(const Node& lhs, const Node& rhs)
  {
    return lhs.cost > rhs.cost;
  }
};

std::pair<int, std::string> findPathToGoal(const Grid<char> &maze)
{
  Position start_position = findStart(maze);
  int start_cost = 0;

  /******************************
   *  TODO TODO TODO TODO TODO
   *  TODO TODO TODO TODO TODO
   *  TODO TODO TODO TODO TODO
   *  TODO TODO TODO TODO TODO
   *  TODO TODO TODO TODO TODO
   *  TODO TODO TODO TODO TODO
   *  TODO TODO TODO TODO TODO
   *  TODO TODO TODO TODO TODO
   ******************************/

  return std::pair<int, std::string>(0, "R");
}


int main()
{
  /** Map from movement characters (U, D, R, L) to position delta's */
  movements['U'] = Position( 0,-1);
  movements['D'] = Position( 0, 1);
  movements['R'] = Position( 1, 0);
  movements['L'] = Position(-1, 0);

  /** Map from tile types ('#', '.', 'S' ...) to cost of moving onto that tile */
  costs['S'] = 10;
  costs['G'] = 3;
  costs['g'] = 0;
  costs['.'] = 2;
  costs[' '] = 1;

  std::auto_ptr<Grid<char> > maze(loadMaze(std::cin));

  std::pair<int, std::string> result = findPathToGoal(*maze);
  int cost = result.first;
  std::string path = result.second;

  std::cout << "Result:" << std::endl;
  plotPath(*maze, path);
  std::cout << "Cost: " << cost << std::endl;
  std::cout << "Path: "  << path << std::endl;

  return 0;
}
