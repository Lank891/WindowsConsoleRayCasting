#include <Windows.h>

#include <array>
#include <chrono>
#include <sstream>
#include <string>
#include <utility>

// Constants
constexpr double pi = 3.141592;
constexpr double rayStepForward = 0.1;
constexpr double maxRayDepth = 16.0;
constexpr double boundariesTreshold = 0.01;

// Constants related to graphics size
constexpr short xBuffer = 220;
constexpr short yBuffer = 120;
constexpr short fontWidth = 8;
constexpr short fontHeight = 8;
constexpr int bufferSize = xBuffer * yBuffer;

// Shade: Lower index = whiter element
constexpr wchar_t floorShadow[] = L"#+.- ";
constexpr wchar_t wallShadow[] = L"\x2588\x2593\x2592\x2591\x0020";

// Constants related to control
constexpr int exitKey = VK_ESCAPE;
constexpr int forwardKey = 'W';
constexpr int backwardKey = 'S';
constexpr int leftRotateKey = 'A';
constexpr int rightRotateKey = 'D';

// Structures for player and map
struct Player {
  double x = 11.0;
  double y = 8.0;
  double angle = -pi / 2;
  double fov = pi / 4;
  double coordSpeed = 2.5;
  double angleSpeed = pi / 3;
};
struct Map {
  int h = 17;
  int w = 23;
  std::wstring map =
      L"#######################"
      L"#           ##    #   #"
      L"#####  ###  #  #  #   #"
      L"#  #           #  #   #"
      L"#  #    ### ######### #"
      L"#  #    ###           #"
      L"#  #    ###       #   #"
      L"#           #######   #"
      L"#  ####           #   #"
      L"#           #######   #"
      L"######      #######   #"
      L"#    #            #   #"
      L"###  #  ######  #######"
      L"#    #                #"
      L"#    #####   ##    #  #"
      L"###                   #"
      L"#######################";
};

// Prepare console and return handles to output and input as a pair
[[nodiscard]] std::pair<HANDLE, HANDLE> PrepareConsole(short x,
                                                       short y,
                                                       short fontw,
                                                       short fonth) {
  HANDLE consoleOut = GetStdHandle(STD_OUTPUT_HANDLE);
  HANDLE consoleIn = GetStdHandle(STD_INPUT_HANDLE);

  COORD bufSize{x, y};
  SetConsoleScreenBufferSize(consoleOut, bufSize);

  SetConsoleActiveScreenBuffer(consoleOut);

  CONSOLE_FONT_INFOEX cfi{sizeof(cfi), 0,         fontw,      fonth,
                          FF_DONTCARE, FW_NORMAL, L"Consolas"};
  SetCurrentConsoleFontEx(consoleOut, false, &cfi);

  SMALL_RECT rectWindow = {0, 0, x - 1, y - 1};
  SetConsoleWindowInfo(consoleOut, TRUE, &rectWindow);

  SetConsoleMode(consoleIn, ENABLE_EXTENDED_FLAGS | ENABLE_WINDOW_INPUT |
                                ENABLE_MOUSE_INPUT);

  SetWindowPos(GetConsoleWindow(), HWND_TOP, 0, 0, 0, 0, SWP_NOSIZE);

  return std::make_pair(consoleOut, consoleIn);
}

// Takes buffer and draws it to the console output handle, returns amount of
// characters drawn
DWORD DrawBufferToCosnole(HANDLE console, wchar_t* buf, const int size) {
  DWORD written;
  WriteConsoleOutputCharacterW(console, buf, size, {0, 0}, &written);
  return written;
}

// Takes player and map and draw them to the buffer in upper left corner
void DrawPlayerAndMapToBuffer(Player& player,
                              Map& map,
                              wchar_t* buf,
                              const int bufferRowSize) {
  for (int i = 0; i < map.h; i++) {
    for (int j = 0; j < map.w; j++) {
      buf[i * bufferRowSize + j] = map.map[i * map.w + j];
    }
  }
  buf[(int)player.x * bufferRowSize + (int)player.y] = L'O';
}

// Takes player, map, column and amount of columns and returns pair: distance to
// hit wall and if it is boundary
[[nodiscard]] std::pair<double, bool> CalculateColumn(Player& player,
                                                      Map& map,
                                                      const int column,
                                                      const int maxColumns) {
  double rayAngle = (player.angle - player.fov / 2.0) +
                    ((double)column / (double)maxColumns) * player.fov;

  double distanceToWall = 0.0;

  double eyeX = sin(rayAngle);
  double eyeY = cos(rayAngle);

  bool isBoundary = false;

  while (distanceToWall < maxRayDepth) {
    int actualBlockY = (int)(player.x + eyeX * distanceToWall);
    int actualBlockX = (int)(player.y + eyeY * distanceToWall);

    // Test if ray is out of the map - then set it is max possible distance
    if (actualBlockX < 0 || actualBlockX >= map.w || actualBlockY < 0 ||
        actualBlockY >= map.h) {
      distanceToWall = maxRayDepth;
      break;
    }

    // Check if we hit wall, check if it is a boundary, then break
    if (map.map[actualBlockY * map.w + actualBlockX] == L'#') {
      // Array for 4 boundaries
      std::array<std::pair<double, double>, 4> boundaries;

      for (int tx = 0; tx < 2; tx++) {
        for (int ty = 0; ty < 2; ty++) {
          // Angle of corner to eyes
          double vY = (double)actualBlockX + ty - player.y;
          double vX = (double)actualBlockY + tx - player.x;
          double dist = sqrt(vX * vX + vY * vY);
          double dot = (eyeX * vX / dist) + (eyeY * vY / dist);
          boundaries.at(2 * tx + ty) = std::make_pair(dist, dot);
        }
      }

      // Sort boundaries to get closest ones
      std::sort(boundaries.begin(), boundaries.end(),
                [](const std::pair<double, double>& left,
                   const std::pair<double, double>& right) {
                  return left.first < right.first;
                });

      if (acos(boundaries.at(0).second) < boundariesTreshold ||
          acos(boundaries.at(1).second) < boundariesTreshold) {
        isBoundary = true;
      }

      break;
    }
    distanceToWall += rayStepForward;
  }

  return std::make_pair(distanceToWall, isBoundary);
}

// Takes column, distance to closest wall in that column, is this coulmn
// boundary of a wall and buffer and draws that colun in the buffer
void DrawColumnToBuffer(const int column,
                        const double wallDistance,
                        const bool isBoundary,
                        wchar_t* buf,
                        const int bufferRowSize,
                        const int bufferColumnSize) {
  // Find ceil and floor heights
  int ceil =
      (int)((bufferColumnSize / 2.0) - (bufferColumnSize / wallDistance));
  int floor = bufferColumnSize - ceil;

  // Pixel for drawing wall and floor
  wchar_t pixelChar;

  // For each pixel in a column
  for (int y = 0; y < bufferColumnSize; y++) {
    if (y <= ceil) {
      // Ceil - just empty space
      buf[y * bufferRowSize + column] = L' ';
    } else if (y >= floor) {
      // Floor - based on how far away the wall is (so how high the floor is)
      // use appropriate shadow
      double floorHeight =
          2.0 * ((double)bufferColumnSize - y) / (double)bufferColumnSize;

      if (floorHeight < 0.25)
        pixelChar = floorShadow[0];
      else if (floorHeight < 0.5)
        pixelChar = floorShadow[1];
      else if (floorHeight < 0.75)
        pixelChar = floorShadow[2];
      else if (floorHeight < 0.9)
        pixelChar = floorShadow[3];
      else
        pixelChar = floorShadow[4];

      buf[y * bufferRowSize + column] = pixelChar;
    } else {
      // Wall - based on how far away wall is use appropriate shadow
      if (wallDistance < maxRayDepth / 4.0)
        pixelChar = wallShadow[0];
      else if (wallDistance < maxRayDepth / 3.0)
        pixelChar = wallShadow[1];
      else if (wallDistance < maxRayDepth / 2.0)
        pixelChar = wallShadow[2];
      else if (wallDistance < maxRayDepth / 1.0)
        pixelChar = wallShadow[3];
      else
        pixelChar = wallShadow[4];

      // If column is a boundary it should be darkest
      if (isBoundary)
        pixelChar = L'I';  // wallShadow[4];

      buf[y * bufferRowSize + column] = pixelChar;
    }
  }
}

// Takes player, map and elapsed time, cheks input and returns if key to exit
// was pressed
[[nodiscard]] bool processInput(Player& player,
                                Map& map,
                                const long long timePassed) {
  // Delta T in seconds instead of miliseconds
  double dt = timePassed / 1000000.0;

  // Forwards movement
  if (GetAsyncKeyState(forwardKey) & 0x8000) {
    player.x += sin(player.angle) * player.coordSpeed * dt;
    player.y += cos(player.angle) * player.coordSpeed * dt;

    if (map.map[(int)player.x * map.w + (int)player.y] == L'#') {
      player.x -= sin(player.angle) * player.coordSpeed * dt;
      player.y -= cos(player.angle) * player.coordSpeed * dt;
    }
  }

  // Backward movement
  if (GetAsyncKeyState(backwardKey) & 0x8000) {
    player.x -= sin(player.angle) * player.coordSpeed * dt;
    player.y -= cos(player.angle) * player.coordSpeed * dt;

    if (map.map[(int)player.x * map.w + (int)player.y] == L'#') {
      player.x += sin(player.angle) * player.coordSpeed * dt;
      player.y += cos(player.angle) * player.coordSpeed * dt;
    }
  }

  // Left rotation
  if (GetAsyncKeyState(leftRotateKey) & 0x8000) {
    player.angle -= player.angleSpeed * dt;
  }

  // Right rotation
  if (GetAsyncKeyState(rightRotateKey) & 0x8000) {
    player.angle += player.angleSpeed * dt;
  }

  // Normalize angle
  if (player.angle < 0)
    player.angle += 2 * pi;
  else if (player.angle > 2 * pi)
    player.angle -= 2 * pi;

  // If exit key is pressed return true (end main loop), otherwise return false
  return GetAsyncKeyState(exitKey) & 0x8000 ? true : false;
}

int main() {
  wchar_t* buffer = new wchar_t[bufferSize];
  Player player;
  Map map;

  auto [consoleOut, consoleIn] =
      PrepareConsole(xBuffer, yBuffer, fontWidth, fontHeight);

  // For time
  auto start = std::chrono::high_resolution_clock::now();
  long long passedMs = 0;
  decltype(start) stop;
  std::wstringstream title;

  while (true) {
    // Timing logic + settin fps counter
    stop = std::chrono::high_resolution_clock::now();
    passedMs =
        std::chrono::duration_cast<std::chrono::microseconds>(stop - start)
            .count();
    long long fps = (long long)10e6 / (passedMs == 0 ? 1 : passedMs);
    title.str(L"");
    title << L"FPS: " << fps << L"; Player coords: " << player.x << L", "
          << player.y << "; Angle: " << 2 * pi - player.angle;
    SetConsoleTitleW(title.str().c_str());
    start = stop;

    // Input Logic
    if (processInput(player, map, passedMs))
      break;

    // Drawing Logic
    for (int x = 0; x < xBuffer; x++) {
      auto [distanceToWall, isBoundary] =
          CalculateColumn(player, map, x, xBuffer);
      DrawColumnToBuffer(x, distanceToWall, isBoundary, buffer, xBuffer,
                         yBuffer);
    }
    DrawPlayerAndMapToBuffer(player, map, buffer, xBuffer);
    DrawBufferToCosnole(consoleOut, buffer, bufferSize);
  }

  delete[] buffer;
}
