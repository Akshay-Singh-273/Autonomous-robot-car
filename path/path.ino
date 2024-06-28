#include <Arduino.h>
#include <queue>
#define SOUND_SPEED 0.034

const int trig1 = 22;
const int echo1 = 23;

int dd_unit = 300;
int td_unit = 300;

long duration;
float distance1;

std::vector<std::vector<int>> grid = {
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0}
};

struct Node {
    int x, y;
    Node* parent;
    int g, h, f;
};

bool operator<(const Node& a, const Node& b) {
    return a.f > b.f;
}


std::vector<std::vector<bool>> closed(grid.size(), std::vector<bool>(grid[0].size(), false));
std::vector<std::vector<Node*>> cameFrom(grid.size(), std::vector<Node*>(grid[0].size(), nullptr));

void resetArrays() {
    closed.assign(grid.size(), std::vector<bool>(grid[0].size(), false));
    cameFrom.assign(grid.size(), std::vector<Node*>(grid[0].size(), nullptr));
}

std::vector<std::vector<int>> directions = {
    {0, -1}, {0, 1}, {-1, 0}, {1, 0}
};

/*
bool isValid(int x, int y, Node grid[GRID_SIZE_X][GRID_SIZE_Y]) {
    return x >= 0 && x < GRID_SIZE_X && y >= 0 && y < GRID_SIZE_Y;
}
*/
std::vector<int> AStar(int startX, int startY, int endX, int endY) {
    std::priority_queue<Node*> open;
    Node* startNode = new Node{startX, startY, nullptr, 0, 0, 0};
    startNode->h = abs(endX - startX) + abs(endY - startY);
    startNode->f = startNode->g + startNode->h;
    open.push(startNode);

    while (!open.empty()) {
        Node* current = open.top();
        open.pop();

        if (current->x == endX && current->y == endY) {
            std::vector<int> path;
            while (current != nullptr) {
                path.push_back(current->x);
                path.push_back(current->y);
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        closed[current->x][current->y] = true;

        for (auto& dir : directions) {
            int newX = current->x + dir[0];
            int newY = current->y + dir[1];
            if (newX < 0 || newX >= grid.size() || newY < 0 || newY >= grid[0].size() || grid[newX][newY] == 1 || closed[newX][newY]) {
                continue;
            }

            Node* neighbor = new Node{newX, newY, current, current->g + 1, 0, 0};
            neighbor->h = abs(endX - newX) + abs(endY - newY);
            neighbor->f = neighbor->g + neighbor->h;

            if (cameFrom[newX][newY] == nullptr || neighbor->f < cameFrom[newX][newY]->f) {
                open.push(neighbor);
                cameFrom[newX][newY] = neighbor;
            }
        }
    }

    return std::vector<int>();
}

void setup()
{
  pinMode(0, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(21, OUTPUT);
  digitalWrite(0, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(16, HIGH);
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  digitalWrite(21, HIGH);
  Serial.begin(9600);
  Serial.println("running program!!");

  
  delay(2000);
  //std::pair<int, int> start = {0, 4};
  //std::pair<int, int> end = {8, 4};

  while (true)
  {
    int i = 0, j = 2;
    Serial.println("one complete");
    while (i < 8) 
    {
      delay(600);
      Serial.println("");
      Serial.print("current position: (");
      Serial.print(i);
      Serial.print(",");
      Serial.print(j);
      Serial.println(")");
      //mark current location
      grid[i][j] = 2;
  
      // Read ultrasonic sensor
      digitalWrite(trig1, LOW);
      delayMicroseconds(2);
      digitalWrite(trig1, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig1, LOW);
      float duration = pulseIn(echo1, HIGH);
      float distance = duration * 0.034 / 2;

      // Update grid based on distance
      if (distance < 25)
      {
        Serial.println("");
        Serial.print("obstruction found at: (");
        Serial.print(i+1);
        Serial.print(",");
        Serial.print(j);
        Serial.println(")");
        grid[i+1][j] = 1;  // Obstacle
      }

      for(int l = 0; l < 8; l++)
      {
        Serial.println("");
        for(int k = 0; k < 5; k++)
        {
          Serial.print(grid[l][k]);
        }
      }
      //std::vector<std::pair<int, int>> path = astar(grid, start, end);

      std::vector<int> path = AStar(i, j, 7, 2);

      if (!path.empty()) {
        for (size_t m = 2; m < path.size(); m += 2) {
            Serial.print(path[m + 1]);
            Serial.print(", ");
            Serial.println(path[m]);
        }
    } else {
        Serial.println("No path found");
    }

        //updateGrid(i,j);
      if (!path.empty())
      {
        //Serial.println("Path found:");
          if(path[3] == i && path[2] < j)
          {
            Serial.println("right");
            digitalWrite(0, HIGH);
            digitalWrite(4, LOW);
            digitalWrite(16, HIGH);
            delay(td_unit);
            digitalWrite(0, HIGH);
            digitalWrite(4, HIGH);
            digitalWrite(16, HIGH);
            delay(td_unit);
            //moving forward
            digitalWrite(0, LOW);
            digitalWrite(4, LOW);
            digitalWrite(16, HIGH);
            delay(dd_unit);
            digitalWrite(0, HIGH);
            digitalWrite(4, HIGH);
            digitalWrite(16, HIGH);
            //facing forward
            delay(td_unit);
            digitalWrite(0, HIGH);
            digitalWrite(4, LOW);
            digitalWrite(16, LOW);
            delay(td_unit);
            digitalWrite(0, HIGH);
            digitalWrite(4, HIGH);
            digitalWrite(16, HIGH);
            // update current location
            j--;
          }
          else if(path[3] == i && path[2] > j)
          {
            Serial.println("left");
            digitalWrite(0, HIGH);
            digitalWrite(4, LOW);
            digitalWrite(16, LOW);
            delay(td_unit);
            digitalWrite(0, HIGH);
            digitalWrite(4, HIGH);
            digitalWrite(16, HIGH);
            delay(td_unit);
            //moving forward
            digitalWrite(0, LOW);
            digitalWrite(4, LOW);
            digitalWrite(16, HIGH);
            delay(dd_unit);
            digitalWrite(0, HIGH);
            digitalWrite(4, HIGH);
            digitalWrite(16, HIGH);
            delay(td_unit);
            //facing forward
            digitalWrite(0, HIGH);
            digitalWrite(4, LOW);
            digitalWrite(16, HIGH);
            delay(td_unit);
            digitalWrite(0, HIGH);
            digitalWrite(4, HIGH);
            digitalWrite(16, HIGH);
            //update current location
            j++;
          }
          else if(path[3] > i && path[2] == j)
          {
            Serial.println("forward");
            digitalWrite(0, LOW);
            digitalWrite(4, LOW);
            digitalWrite(16, HIGH);
            delay(dd_unit);
            digitalWrite(0, HIGH);
            digitalWrite(4, HIGH);
            digitalWrite(16, HIGH);
            //update current location
            i++;
          }
          //Serial.print("(");
          //Serial.print(it->first);
          //Serial.print(", ");
          //Serial.print(it->second);
          //Serial.println(")");
          digitalWrite(0, HIGH);
          digitalWrite(4, HIGH);
          digitalWrite(16, HIGH);
          resetArrays(); // Reset the closed and cameFrom arrays
          
        }
        //delay(800);
      }
      // Delay before next update
    }
  }
/*
    std::vector<std::pair<int, int>> path = astar(grid, start, end);

    if (!path.empty()) {
        Serial.println("Path found:");
        for (auto it = path.rbegin(); it != path.rend(); ++it) {
            Serial.print("(");
            Serial.print(it->first);
            Serial.print(", ");
            Serial.print(it->second);
            Serial.println(")");
        }
    } else {
        Serial.println("No path found");
    }
}
*/

void loop() {
}
