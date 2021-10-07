/**
 * @file LibPotentialFields.h
 *
 * This file defines a representation that checks some behavior control properties
 *
 * @author Daniel Krause
 */

#pragma once

#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Modeling/NodePF.h"
#include "Tools/Function.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Representations/Modeling/ObstacleModel.h"

#define DISTANCE(x1, y1, x2, y2) sqrt(pow(x1-x2,2)+pow(y1-y2,2))

//TODO check that SummableType implements basic arithmetic operators
//template <typename SummableType> 

//TODO implement all derived classes for each kind of field component 
//(including Custom ones with custom fieldLaw functions, provided as an argument and used by the overridden computeField function)
struct FieldComponent
{
  public:
    //using ValueType = SummableType;
    
    ENUM(Type,
    {,
      attractive,
      repulsive,
      customType,
    });

    ENUM(IntensityLaw,
    {,
      constant,
      linear,
      quadratic,
      linearThenQuadratic,
      customLaw,
    });

    struct FieldParameters
    {
      public:
        float RO;
        float Kap;
        float Kbp;
        float Kr;
        float ETA; 
        float GAMMA;

        FieldParameters(float RO = 0, float Kap = 0, float Kbp = 0, float Kr = 0, float ETA = 0, float GAMMA = 0)
        : RO(RO), Kap(Kap), Kbp(Kbp), Kr(Kr), ETA(ETA), GAMMA(GAMMA) {}
    };

  protected:
    Vector2f source;
    int innerRadius;
    int outerRadius;
    const Type fieldType;
    const IntensityLaw fieldIntensity;

    FieldComponent(const Vector2f& source,
                  const Type& fieldType, const IntensityLaw& fieldIntensity, 
                  int innerRadius = 0, int outerRadius = 0)
    : source(source), fieldType(fieldType), fieldIntensity(fieldIntensity), innerRadius(innerRadius), outerRadius(outerRadius)
    {
      ASSERT(innerRadius>=0 && outerRadius>=0 and outerRadius>=innerRadius);
    }
  
  public:

    //The field computation function has to be specialized
    virtual std::vector<std::vector<Vector2f>> computeField() = 0;
};

struct PotentialField
{
  protected:
    PotentialField(int cell_size, std::vector<std::vector<Vector2f>> cells, std::vector<float> xCoordinates, std::vector<float> yCoordinates, Vector2f fieldCenter) 
    : cell_size(cell_size), cells(cells), xCoordinates(xCoordinates), yCoordinates(yCoordinates), fieldCenter(fieldCenter)
    {}

    Vector2f getPotential(int x, int y)
    {
      if(isOutOfBounds(x,y))
//TODO Improve this exception message adding x,y coordinates and actual field range ([X_LOWER_BOUND, X_UPPER_BOUND], [Y_LOWER_BOUND, Y_UPPER_BOUND])
        throw std::out_of_range("Selected coordinates out of field range");

      Vector2i cellIndices = getIndicesFromCoordinates(x, y);

      try {
        return cells[cellIndices.x()][cellIndices.y()];
      } 
      catch(const std::exception& e) {
        std::rethrow_exception(std::current_exception());
      }
    }

  public:
    const float cell_size;
    const std::vector<FieldComponent> components;
    const Vector2f fieldCenter;

    Vector2f setPotential(int x, int y, Vector2f& newPotential)
    {
      if(isOutOfBounds(x,y))
//TODO Improve this exception message adding x,y coordinates and actual field range ([X_LOWER_BOUND, X_UPPER_BOUND], [Y_LOWER_BOUND, Y_UPPER_BOUND])
        throw std::out_of_range("Selected coordinates out of field range");

      Vector2i cellIndices = getIndicesFromCoordinates(x, y);
      
      try {
        cells[x][y].x() = newPotential.x();
        cells[x][y].y() = newPotential.y();
      } 
      catch(const std::exception& e) {
        std::rethrow_exception(std::current_exception());
      }
    }

    virtual bool isOutOfBounds(float x, float y);

    virtual bool isCircular();

    private:
      std::vector<std::vector<Vector2f>> cells;
      
      //xCoordinates will contain the SPARSE x coordinates of each cell of that column (xCoordinates[0], of length X_CELLS, will be the x coordinates of the leftmost column of cells)      
      std::vector<float> xCoordinates;
      
      //yCoordinates will contain the SPARSE y coordinates of each cell of that row (yCoordinates[0], of length X_CELLS, will be the y coordinates of the uppermost row of cells)
      std::vector<float> yCoordinates;

      Vector2i getIndicesFromCoordinates(int x, int y)
      {
//TODO this function, given x,y coordinates of a point on the field, will find which i,j indices correspond
//to the cell containing those coordinates, based on the xCoordinates and yCoordinates arrays
      }

//TODO implement an addComponent function that recomputes the potential field based on last addition
//TODO implement an operator[] function (just calling getPotential)
//TODO implement an operator+ and operator- function that sums potential fields

//TODO implement a normalization function that normalizes all potentials to unit vectors (when potential strength is not needed)

//TODO implement a print function

//TODO implement a getNeighbors function that returns all (max 8) neighboring cells to a given x,y coordinate
//TODO implement a getNeighborInDirection function that, given a direction (Vector2f) returns the neighboring cell in that direction, if there is one, otherwise raises an exception

//TODO IMPORTANT implement a destroyer that FREES memory
};

struct SquarePotentialField : PotentialField
{
  public:
    const int X_CELLS;
    const int Y_CELLS;

    static SquarePotentialField initialize(Vector2f fieldCenter, int X_CELLS, int Y_CELLS, int CELL_SIZE, 
                                          Vector2f initialPotential = Vector2f(0,0))
    {
//TODO INITIALIZATION LOGIC: 
//xCoordinates will contain the SPARSE x coordinates of each cell of that column (xCoordinates[0], of length X_CELLS, will be the x coordinates of the leftmost column of cells)
//yCoordinates will contain the SPARSE y coordinates of each cell of that row (yCoordinates[0], of length X_CELLS, will be the y coordinates of the uppermost row of cells)
      ASSERT(X_CELLS>0);
      ASSERT(Y_CELLS>0);
      
      std::vector<float> xCoordinates;
      std::vector<float> yCoordinates;

      std::vector<std::vector<Vector2f>> cells;
      for(int i = 0; i < X_CELLS; i++)
      {
        std::vector<Vector2f> column; 
        for(int j=0; j < Y_CELLS; j++)
        {
          column[j] = Vector2f(initialPotential.x(), initialPotential.y());
        }
        cells[i] = column;
      }

      return SquarePotentialField(cells, xCoordinates, yCoordinates, fieldCenter, X_CELLS, Y_CELLS, CELL_SIZE, initialPotential);
    }

  private:
    SquarePotentialField(std::vector<std::vector<Vector2f>> cells, std::vector<float> xCoordinates, std::vector<float> yCoordinates, 
                        Vector2f fieldCenter, int X_CELLS, int Y_CELLS, int cell_size, 
                        Vector2f initialPotential)
    : PotentialField(cell_size, cells, xCoordinates, yCoordinates, fieldCenter) , 
    X_CELLS(X_CELLS), Y_CELLS(Y_CELLS)
    {}

//TODO not sure, TEST
    bool isOutOfBounds(float x, float y) override
    {
      return x<fieldCenter.x() + X_CELLS/2 && x>fieldCenter.x() - X_CELLS/2 
            && y<fieldCenter.y() + Y_CELLS/2 && y>fieldCenter.y() - Y_CELLS/2;
    }

};

struct CircularPotentialField : PotentialField
{
  public:
    const float RADIUS;

    static CircularPotentialField initialize(Vector2f fieldCenter, float RADIUS, int CELL_SIZE, 
                                          Vector2f initialPotential = Vector2f(0,0))
    {
      ASSERT(RADIUS>0);
      
//TODO INITIALIZATION LOGIC: 
//xCoordinates will contain the SPARSE x coordinates of each cell of that column (xCoordinates[0], of length X_CELLS, will be the x coordinates of the leftmost column of cells)
//yCoordinates will contain the SPARSE y coordinates of each cell of that row (yCoordinates[0], of length X_CELLS, will be the y coordinates of the uppermost row of cells)
//Keep in mind this is a circle so the sparseness here comes in handy. Cells will be initialized if their distance from the center is below radius

      //To build this Potential field we are going to initialize all ceil(RADIUS/CELL_SIZE) cells of size CELL_SIZE
      //contained in the square [RADIUS, RADIUS] and also contained within the circle of center fieldCenter and radius RADIUS
      const int X_CELLS = (int) ceil(RADIUS/CELL_SIZE);
      int Y_CELLS = X_CELLS;

      std::vector<float> xCoordinates;
      std::vector<float> yCoordinates;

//TODO add control on distance vs circle radius
      std::vector<std::vector<Vector2f>> cells;
      for(int i = 0; i < X_CELLS; i++)
      {
        std::vector<Vector2f> column; 
        for(int j=0; j < Y_CELLS; j++)
        {
          column[j] = Vector2f(initialPotential.x(), initialPotential.y());
        }
        cells[i] = column;
      }
    
      return CircularPotentialField(cells, xCoordinates, yCoordinates, fieldCenter, RADIUS, CELL_SIZE, initialPotential);
    }

  private:
    CircularPotentialField(std::vector<std::vector<Vector2f>> cells, std::vector<float> xCoordinates, std::vector<float> yCoordinates, 
                        Vector2f fieldCenter, float RADIUS, int cell_size, 
                        Vector2f initialPotential)
    : PotentialField(cell_size, cells, xCoordinates, yCoordinates, fieldCenter) , 
    RADIUS(RADIUS)
    {}

//TODO not sure, TEST
    bool isOutOfBounds(float x, float y) override
    {
      return DISTANCE(x,y,fieldCenter.x(),fieldCenter.y()) < RADIUS;
    }

    bool isCircular() override { return true; }
};

/*
struct AttractiveComponent : public FieldComponent
{
  public:

};*/

STREAMABLE(LibPotentialFields,
{

  /** Provides the distance between 2 Pose2f **/
  FUNCTION(float(Pose2f p1, Pose2f p2)) distance;

  /** Computes the attractive field for the striker **/
  FUNCTION(std::vector<NodePF>(std::vector<NodePF>& potential_field, Vector2f goal, float RO, float Kap, float Kbp, float Kr,
                                                    float TEAMMATE_CO, float ETA, float GAMMA)) computeStrikerAttractivePF;
  
  /** Computes the repulsive field for the striker specifying a custom obstacle list **/
  FUNCTION(std::vector<NodePF>(std::vector<NodePF>& potential_field, Vector2f source_pos, std::vector<Vector2f>& repulsive_obstacles, float RO, float Kap, float Kbp, float Kr,
                                                    float TEAMMATE_CO, float ETA, float GAMMA, float POW)) computeStrikerRepulsivePFWithCustomObstacles;

  /** Computes the repulsive field for the striker **/
  FUNCTION(std::vector<NodePF>(std::vector<NodePF>& potential_field, Vector2f source_pos, bool navigateAroundBall, float RO, float Kap, float Kbp, float Kr,
                                                    float TEAMMATE_CO, float ETA, float GAMMA, float POW)) computeStrikerRepulsivePF;

  /** Initializes an empty PF spanning the whole field **/
  FUNCTION(std::vector<NodePF>(float cell_size, float FIELD_BORDER_OFFSET)) initializePFAllField;

  /** Initializes an empty PF around a certain point, with a specified radius **/
  FUNCTION(std::vector<NodePF> (float cell_size, Vector2f field_center, float field_radius, float FIELD_BORDER_OFFSET)) initializePFAroundPoint;

  /** Computes an artificial potential field based on the provided attractive and repulsive fields **/
  FUNCTION(std::vector<NodePF>(std::vector<NodePF>& potential_field, std::vector<NodePF> attractive_field, std::vector<NodePF> repulsive_field)) computePFAllField;

  /** Computes an artificial potential field based on the provided attractive and repulsive fields, in a certain radius given a center point **/
  FUNCTION(std::vector<NodePF>(std::vector<NodePF>& potential_field, std::vector<NodePF> attractive_field, std::vector<NodePF> repulsive_field)) computePFAroundPoint;

  FUNCTION(float(float value, float fromIntervalMin, float fromIntervalMax, float toIntervalMin, float toIntervalMax)) mapToInterval;

  FUNCTION(bool(float currentValue, float target, float bound)) isValueBalanced;
  FUNCTION(float(float x, float y)) angleToTarget;
  FUNCTION(float(float x, float y)) norm;

  FUNCTION(Pose2f(float x, float y)) glob2Rel;
  FUNCTION(float(float x)) radiansToDegree;

  FUNCTION(Pose2f(float x, float y)) rel2Glob,

});
