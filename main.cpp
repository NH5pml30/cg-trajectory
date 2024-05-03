#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <iostream>
#include <vector>
#include <optional>

#include <GL/freeglut.h>

void drawInput();
void drawObstacles();
void drawVisibilityGraph();
void drawShortestPath();
void addPoint(float x, float y);
void removeLastPoint();
void finishPolygon();
void finishPolygons();
void finish();

// Variables for zooming and panning
GLfloat zoomFactor = 1.0f;
GLfloat panX = 0.0f;
GLfloat panY = 0.0f;
bool isDragging = false;
int lastMouseX, lastMouseY;
int windowWidth, windowHeight;

// Function to render the scene
void renderScene() {
  glClearColor(0.3f, 0.5f, 0.7f, 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // Apply zoom and pan transformations
  glTranslatef(panX, panY, 0.0f);
  glScalef(zoomFactor, zoomFactor, 1.0f);

  // Draw input
  drawInput();

  // Draw obstacles
  drawObstacles();

  // Draw visibility graph
  drawVisibilityGraph();

  // Draw shortest path
  drawShortestPath();

  glutSwapBuffers();
}

// Function to handle window resizing
void reshape(int width, int height) {
  windowWidth = width;
  windowHeight = height;
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(0, width, 0, height);
}

// Function to handle keyboard input
void keyboard(unsigned char key, int x, int y) {
  switch (key) {
  case 27: // ESC key
    exit(0);
    break;
  case ' ':
    finishPolygon();
    glutPostRedisplay(); // Request redraw
    break;
  case 'Z':
    removeLastPoint();
    glutPostRedisplay(); // Request redraw
    break;
  case 'P':
    finishPolygons();
    glutPostRedisplay(); // Request redraw
    break;
  case 'C':
    finish();
    glutPostRedisplay(); // Request redraw
    break;
  default:
    break;
  }
}

// Function to handle mouse click events
void mouseClick(int button, int state, int x, int y) {
  if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
    // Start dragging
    isDragging = true;
    lastMouseX = x;
    lastMouseY = y;
  } else if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
    // Stop dragging
    isDragging = false;
  } else if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {
    addPoint((x - panX) / zoomFactor,
             ((windowHeight - y - 1) - panY) / zoomFactor);
    glutPostRedisplay(); // Request redraw
  }
}

// Function to handle mouse drag events
void mouseDrag(int x, int y) {
  if (isDragging) {
    // Calculate delta mouse movement
    int deltaX = x - lastMouseX;
    int deltaY = y - lastMouseY;

    // Update pan
    panX += deltaX;
    panY -= deltaY; // Y-axis is inverted in screen coordinates

    // Update last mouse position
    lastMouseX = x;
    lastMouseY = y;

    glutPostRedisplay(); // Request redraw
  }
}

// Function to handle mouse wheel events
void mouseWheel(int wheel, int direction, int x, int y) {
  if (direction > 0) {
    // Zoom in
    zoomFactor *= 1.1f;
  } else {
    // Zoom out
    zoomFactor /= 1.1f;
  }

  glutPostRedisplay(); // Request redraw
}

#pragma comment(lib, "gmp.lib")
#pragma comment(lib, "mpfr.lib")

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Segment_2 Segment_2;
typedef CGAL::Arr_segment_traits_2<Kernel> Traits_2;
typedef CGAL::Arrangement_2<Traits_2> Arrangement_2;
typedef Arrangement_2::Vertex_handle Vertex_handle;
typedef Arrangement_2::Vertex_const_handle Vertex_const_handle;
typedef Arrangement_2::Halfedge_handle Halfedge_handle;

using graph_weight_t = double;
typedef boost::adjacency_list<
    boost::vecS, boost::vecS, boost::undirectedS, boost::no_property,
    boost::property<boost::edge_weight_t, graph_weight_t>>
    Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;

// Function to compute visibility graph
void computeVisibilityGraph(Vertex_const_handle start, Vertex_const_handle end,
                            const Arrangement_2 &obstacles, Graph &graph,
                            std::vector<Point_2> &vertex2point) {
  // Compute visibility graph
  auto for_each_ccb_vertex =
      [](Arrangement_2::Ccb_halfedge_const_circulator ccb, auto &&callback) {
        auto cur = ccb;
        do {
          callback(cur);
        } while (++cur != ccb);
      };
  auto check_points =
      [&](Vertex_const_handle pt_1, int i_1,
          std::optional<Arrangement_2::Ccb_halfedge_const_circulator> c_1,
          int ci_1, Vertex_const_handle pt_2, int i_2,
          std::optional<Arrangement_2::Ccb_halfedge_const_circulator> c_2,
          int ci_2) {
    if (pt_1 == pt_2)
      return;

    Segment_2 view(pt_1->point(), pt_2->point());
    bool is_edge = false;
    bool visible = true;

    if (c_1.has_value() && c_2.has_value() && ci_1 == ci_2) {
      // same ccb
      auto &c_1v = *c_1;
      auto &c_2v = *c_2;
      if (c_2v == std::next(c_1v)) {
        // Add outer edge
        is_edge = true;
        visible = false;
      } else {
        // Check if this diagonal is obstacle-interior
        auto c_1_next = std::next(c_1v);
        auto &A = c_1v->source()->point();
        auto &B = c_1v->target()->point();
        auto &C = c_1_next->target()->point();
        auto &D = pt_2->point();
        std::cout << "compare A, B, C, D: " << A << ", " << B << ", " << C << ", " << D << "\n";
        if (CGAL::orientation(A, B, C) == CGAL::Sign::LEFT_TURN) {
          std::cout << "left turn\n";
          if (!(CGAL::orientation(A, B, D) == CGAL::Sign::LEFT_TURN &&
                CGAL::orientation(B, C, D) == CGAL::Sign::LEFT_TURN)) {
            std::cout << pt_1->point() << " - " << pt_2->point()
                      << " is skipped\n";
            visible = false;
          }
        } else {
          std::cout << "right turn\n";
          if (CGAL::orientation(A, B, D) != CGAL::Sign::LEFT_TURN &&
              CGAL::orientation(B, C, D) != CGAL::Sign::LEFT_TURN) {
            std::cout << pt_1->point() << " - " << pt_2->point()
                      << " is skipped\n";
            visible = false;
          }
        }
      }
    }

    for (auto edge = obstacles.edges_begin();
         visible && edge != obstacles.edges_end(); ++edge) {
      if (edge->source() == pt_1 && edge->target() == pt_2 ||
          edge->source() == pt_2 && edge->target() == pt_1) {
        std::cout << pt_1->point() << " is connected with " << pt_2->point()
                  << "\n";
        visible = false;
        break;
      }
      if (edge->source() == pt_1 || edge->target() == pt_1 ||
          edge->source() == pt_2 || edge->target() == pt_2)
        continue;
      if (CGAL::do_intersect(view, Segment_2(edge->source()->point(),
                                             edge->target()->point()))) {
        visible = false;
        break;
      }
    }

    auto add_edge = [&](auto &pt_1, int i_1, auto &pt_2, int i_2) {
      double distance =
          sqrt(CGAL::to_double(CGAL::squared_distance(pt_1->point(), pt_2->point())));
      Edge edge;
      bool success;
      boost::tie(edge, success) = boost::add_edge(i_1, i_2, graph);
      if (success) {
        boost::put(boost::edge_weight, graph, edge, distance);
      }
      std::cout << pt_1->point() << " is visible from " << pt_2->point()
                << " with distance " << distance << "\n";
    };

    if (visible || is_edge) {
      add_edge(pt_1, i_1, pt_2, i_2);
      if (is_edge)
        add_edge(pt_2, i_2, pt_1, i_1);
    }
  };
  auto for_each_map_vertex = [&](auto &&callback) {
    int i = 0;
    auto idx_callback = [&](Arrangement_2::Ccb_halfedge_const_circulator pt,
                            int c) mutable { callback(pt->target(), i++, c, pt); };
    auto outer = obstacles.unbounded_face();
    callback(start, i++, -1, {});
    callback(end, i++, -1, {});
    int c = 0;
    for (auto inner_ccb = outer->inner_ccbs_begin();
         inner_ccb != outer->inner_ccbs_end(); ++inner_ccb) {
      for_each_ccb_vertex(*inner_ccb,
                          [&](Arrangement_2::Ccb_halfedge_const_circulator pt) {
                            idx_callback(pt, c);
                          });
      c++;
    }
  };

  for_each_map_vertex(
      [&](Vertex_const_handle pt_1, int i_1, int ci_1,
          std::optional<Arrangement_2::Ccb_halfedge_const_circulator> c_1) {
        std::cout << ci_1 << ": " << i_1 << ": " << pt_1->point() << std::endl;
        vertex2point.push_back(pt_1->point());

        for_each_map_vertex(
            [&](Vertex_const_handle pt_2, int i_2, int ci_2,
                std::optional<Arrangement_2::Ccb_halfedge_const_circulator>
                    c_2) {
              check_points(pt_1, i_1, c_1, ci_1, pt_2, i_2, c_2, ci_2);
            });
      });
}

// Function to create an arrangement from a vector of polygons
Arrangement_2 createArrangementFromPolygons(
    const std::vector<std::vector<Point_2>> &polygons) {
  Arrangement_2 arrangement;

  for (const auto &polygon : polygons) {
    for (std::size_t i = 0; i < polygon.size(); ++i) {
      Point_2 source = polygon[i];
      Point_2 target =
          polygon[(i + 1) % polygon.size()];
      /* if (!CGAL::lexicographically_xy_smaller(source, target)) {
        std::swap(source, target);
      } */
      Segment_2 segment(source, target);

      CGAL::insert(arrangement, segment);
    }
  }

  return arrangement;
}

// Function to run Dijkstra's algorithm and find the shortest path
void findShortestPath(const Graph &graph, Vertex source, Vertex target,
                      std::vector<Vertex> &predecessor,
                      std::vector<graph_weight_t> &distance) {
  predecessor.resize(boost::num_vertices(graph));
  distance.resize(boost::num_vertices(graph));

  boost::dijkstra_shortest_paths(
      graph, source,
      boost::predecessor_map(
          boost::make_iterator_property_map(
              predecessor.begin(), boost::get(boost::vertex_index, graph)))
          .distance_map(boost::make_iterator_property_map(
              distance.begin(), boost::get(boost::vertex_index, graph))));

  // Output the shortest path
  std::cout << "Shortest path from vertex " << source << " to vertex " << target
            << ":\n";
  std::cout << "Distance: " << distance[target] << "\n";
  std::cout << "Path: ";
  for (Vertex v = target; v != source; v = predecessor[v]) {
    std::cout << v << " ";
  }
  std::cout << source << "\n";
}

Arrangement_2 obstacles;
Graph visibility;
std::vector<Point_2> vertex2point;
std::vector<Vertex> predecessor;
std::vector<graph_weight_t> distance;
std::vector<std::vector<Point_2>> in_polygons;
std::vector<Point_2> cur_polygon;
std::optional<Point_2> start, end;

void addPoint(float x, float y) {
  cur_polygon.push_back({x, y});
}

void removeLastPoint() {
  if (!cur_polygon.empty())
    cur_polygon.pop_back();
}

void finishPolygon() {
  in_polygons.emplace_back();
  in_polygons.back().swap(cur_polygon);
}

void finishPolygons() {
  start = cur_polygon.back();
  cur_polygon.clear();
}

void finish() {
  if (!cur_polygon.empty())
    end = cur_polygon.back();
  cur_polygon.clear();

  #if 0
  start = {-1, -1};
  in_polygons = {
      //{{0, 0}, {1, 0}, {1, 1}, {0, 1}},
      //{{2, 2}, {3, 2}, {3, 3}, {2, 3}}
      // {{0, 0}, {4, 0}, {4, 4}, {0, 4}},
      {{1, 1}, {2, 1}, {2, 2}, {1, 2}},
      {{-0.5, 1.2}, {2.5, 1.2}, {2.5, 1.8}, {-0.5, 1.8}},
      /*{{8, 8}, {12, 8}, {12, 12}, {8, 12}},
      {{9, 9}, {10, 9}, {10, 10}, {9, 10}},*/
  };
  end = {4, 4};
  #endif


  obstacles = createArrangementFromPolygons(in_polygons);
  in_polygons.clear();
  auto start_handle = CGAL::insert_point(obstacles, *start);
  auto end_handle = CGAL::insert_point(obstacles, *end);
  visibility = Graph(obstacles.number_of_vertices());

  // Compute visibility graph
  computeVisibilityGraph(start_handle, end_handle, obstacles, visibility,
                         vertex2point);

  // Find shortest path
  findShortestPath(visibility, 0, 1, predecessor, distance);
}

void drawInput() {
  glLineWidth(1);

  // Draw input edges
  glColor3f(1.0f, 1.0f, 1.0f);
  for (auto &poly : in_polygons) {
    glBegin(GL_LINE_LOOP);
    for (auto &pt : poly)
      glVertex2d(CGAL::to_double(pt.x()), CGAL::to_double(pt.y()));
    glEnd();
  }

  glColor3f(0.7f, 0.7f, 0.7f);
  glBegin(GL_LINE_STRIP);
  for (auto &pt : cur_polygon)
    glVertex2d(CGAL::to_double(pt.x()), CGAL::to_double(pt.y()));
  glEnd();

  // Draw input vertices
  glPointSize(2.0); // Set point size for vertices
  glColor3f(1.0f, 1.0f, 1.0f);
  for (auto &poly : in_polygons) {
    glBegin(GL_POINTS);
    for (auto &pt : poly)
      glVertex2d(CGAL::to_double(pt.x()), CGAL::to_double(pt.y()));
    glEnd();
  }

  glColor3f(0.7f, 0.7f, 0.7f);
  glBegin(GL_POINTS);
  for (auto &pt : cur_polygon)
    glVertex2d(CGAL::to_double(pt.x()), CGAL::to_double(pt.y()));
  glEnd();

  glColor3f(0.0f, 0.0f, 1.0f);
  glBegin(GL_POINTS);
  if (start.has_value())
    glVertex2d(CGAL::to_double(start->x()), CGAL::to_double(start->y()));
  if (end.has_value())
    glVertex2d(CGAL::to_double(end->x()), CGAL::to_double(end->y()));
  glEnd();
}

void drawObstacles() {
  glLineWidth(1);

  glColor3f(1.0f, 1.0f, 1.0f);
  glBegin(GL_LINES);
  for (auto edge = obstacles.edges_begin(); edge != obstacles.edges_end();
       ++edge) {
    Point_2 source = edge->source()->point();
    Point_2 target = edge->target()->point();
    glVertex2d(CGAL::to_double(source.x()), CGAL::to_double(source.y()));
    glVertex2d(CGAL::to_double(target.x()), CGAL::to_double(target.y()));
  }
  glEnd();

  // Draw arrangement vertices
  glPointSize(2.0); // Set point size for vertices
  glBegin(GL_POINTS);
  for (auto vertex = obstacles.vertices_begin();
       vertex != obstacles.vertices_end(); ++vertex) {
    Point_2 point = vertex->point();
    glVertex2d(CGAL::to_double(point.x()), CGAL::to_double(point.y()));
  }
  glEnd();
}

void drawVisibilityGraph() {
  glLineWidth(3);

  glColor3f(0.5f, 1.0f, 0.5f);

  // Draw graph edges
  glBegin(GL_LINES);
  for (auto iter = boost::edges(visibility).first;
       iter != boost::edges(visibility).second; ++iter) {
    Vertex source = boost::source(*iter, visibility);
    Vertex target = boost::target(*iter, visibility);
    Point_2 sourcePoint = vertex2point[source];
    Point_2 targetPoint = vertex2point[target];
    glVertex2d(CGAL::to_double(sourcePoint.x()),
               CGAL::to_double(sourcePoint.y()));
    glVertex2d(CGAL::to_double(targetPoint.x()),
               CGAL::to_double(targetPoint.y()));
  }
  glEnd();

  // Draw graph vertices
  glPointSize(4.0); // Set point size for vertices
  glBegin(GL_POINTS);
  for (const auto &point : vertex2point) {
    glVertex2d(CGAL::to_double(point.x()), CGAL::to_double(point.y()));
  }
  glEnd();
}

void drawShortestPath() {
  if (predecessor.empty())
    return;

  glLineWidth(4);

  glColor3f(1.0f, 0.0f, 0.0f);

  // Draw shortest path edges
  glBegin(GL_LINE_STRIP);
  for (Vertex v = 1; v != 0; v = predecessor[v]) {
    glVertex2d(CGAL::to_double(vertex2point[v].x()),
               CGAL::to_double(vertex2point[v].y()));
  }
  glVertex2d(CGAL::to_double(vertex2point[0].x()),
             CGAL::to_double(vertex2point[0].y()));
  glEnd();

  // Draw shortest path vertices
  glPointSize(5.0); // Set point size for vertices
  glBegin(GL_POINTS);
  for (Vertex v = 1; v != 0; v = predecessor[v]) {
    glVertex2d(CGAL::to_double(vertex2point[v].x()),
               CGAL::to_double(vertex2point[v].y()));
  }
  glVertex2d(CGAL::to_double(vertex2point[0].x()),
             CGAL::to_double(vertex2point[0].y()));
  glEnd();
}

int main(int argc, char *argv[]) {
  // Define your input data (obstacles, start and end vertices)
  std::vector<std::vector<Point_2>> polygons = {
      //{{0, 0}, {1, 0}, {1, 1}, {0, 1}},
      //{{2, 2}, {3, 2}, {3, 3}, {2, 3}}
      {{0, 0}, {4, 0}, {4, 4}, {0, 4}},
      {{1, 1}, {2, 1}, {2, 2}, {1, 2}},
      {{-0.5, 1.2}, {2.5, 1.2}, {2.5, 1.8}, {-0.5, 1.8}},
      /*{{8, 8}, {12, 8}, {12, 12}, {8, 12}},
      {{9, 9}, {10, 9}, {10, 10}, {9, 10}},*/
  };
  Point_2 start = {-1, -1};
  // Point_2 start = {13, 13};
  Point_2 end = {14, 14};

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(800, 600);
  glutCreateWindow("Boost Graph Visualization");

  // Register FreeGLUT callback functions
  glutDisplayFunc(renderScene);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouseClick);
  glutMotionFunc(mouseDrag);
  glutMouseWheelFunc(mouseWheel);

  // Start the FreeGLUT main loop
  glutMainLoop();

  return 0;
}