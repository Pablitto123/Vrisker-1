/*
 * slam.c
 *
 *  Created on: Nov 3, 2020
 *      Author: Łukasz Zelek
 */
#include <slam.h>
#include <I2C_access.h>
// zawiera kod lokalizujący robota w labiryncie z użyciem punktów stałych, (znamy punkty stykania się ścianek)
// na wejście przydaje się pozycja z Kalman [x,y]
// na wyjście dostajemy obliczone [x,y], okazjonalnie mapę otoczenia ściankami
/**
 * TODO BIBLIOTEKA MATEMATYCZNA I FPU
 * Instrukcja wykrywania landmark do ustalenia pozycji przy każdym obrocie robota
 *
 * 1) śledzimy pomiary i odrzucamy wszystkie poza 1-2000 <- stanowią one błędy
 * 2) odkładamy dane w buforze, kolejka, lub stos
 * 3) przeprowadzamy sortowanie kątowe
 * 4)
    Przekształcenie punktów z układu lidarów di na układ robota Pi,
    gdzie (0,0) oś robota

    P1 = (-d1, 0) + (-310, 230)
    P2 = (-d2 * sqrt(2) / 2, d2 * sqrt(2) / 2) + (-180, 330)
    P3 = (0, d3) + (0, 380)
    P4 = (d4 * sqrt(2) / 2, d4 * sqrt(2) / 2) + (180, 330)
    P5 = (d5, 0) + (310, 230)

    Wyprowadzony wzór na przekszałcenie przy układach współrzędnych,
    z układu robota(x,y) na układ świata(px,py)
    przy zmianie pozycji robota o r i kąta o alpha

    px = sqrt(x^2 + y^2) * cos(arctg(y/x) - alpha) - r * cos(alpha)
    py = sqrt(x^2 + y^2) * sin(arctg(y/x) - alpha) - r * sin(alpha)
 *
 * 5) Regresja liniowa od prawej do lewej badanie współliniowości
 * 6) Sprawdzamy punkty przecięcia odnalezionych prostych, to nasze landmarki,
 * update position to known
 * Jazda prosto:
 * 1) Jedziemy, po prostej
 * 2) Widzimy nagły spadek na jednej z diagonali, to nasz landmark,
 * przeprowadzamy update pozycji
 * 3) Oczekujemy na potwierdzenie przez czujniki boczne spadku,
 * również przeprowadzamy update pozycji
 * 4) Widzimy nagły przyrost i póżniejszy spadek, wykryty słupek,
 * aktualizujemy pozycję, to samo na bocznym
 * Jazda na ukos:
 * Wykrywamy słupki i analogiczie aktualizujemy pozycję
*/

#define N_START_MEASUREMENT_CYCLES 10
#define N_SENSORS 5

#define N_WALL_POINTS 1000

#define ASSUMED_DISTANCE_BETWEEN_WALLS (168) /*mm*/
#define dist get_proximity

void transform_from_sensors_to_robot_coordinates(int *sensor, int *data_x, int *data_y);
int cramer(int x1, int y1, int x2, int y2, int *a, int *b);

// to init
int start_alpha;
int start_distance_beetween_walls;
int start_pos_x, start_pos_y;

int pos_x, pos_y;

int measure_avg[N_SENSORS];

int x_buffer[N_WALL_POINTS]; // world coord
int y_buffer[N_WALL_POINTS];

int counter = 0;

int x_data[N_SENSORS];
int y_data[N_SENSORS];
int dist_meas[N_SENSORS];

// defincja prostej w programie
// prosta k (A,B,C)

// zbior prostych (1, 0, 0), (0, 1, 0) jednostka mm
// ogolnie proste mają postać (1, 0, -A), (0, 1, -B) A,B > 0
int a1,a2,b1,b2,result; // temp values
float __x = 90, __y = 90, __alpha = 3*PI/2;

void init_start_position(int *sensor_data, int *start_x, int *start_y){
    for(int i = 0; i < N_START_MEASUREMENT_CYCLES*N_SENSORS; i++)
    {
        measure_avg[i % N_SENSORS] += sensor_data[i];
    }
    for(int i = 0; i < N_SENSORS; i++)
        measure_avg[i] /= N_START_MEASUREMENT_CYCLES;
    transform_from_sensors_to_robot_coordinates(measure_avg, x_data, y_data);
    result = cramer(x_data[0],y_data[0],x_data[1],y_data[1],&a1,&b1);
    if(result != 0)printf("Cramer problem\n");
    result = cramer(x_data[2],y_data[2],x_data[3],y_data[3],&a2,&b2);
    if(result != 0)printf("Cramer problem\n");

    start_alpha = 90 - atanf(a1); // deg to rad
    start_distance_beetween_walls = (fabsf(x_data[0])+fabsf(x_data[4]))*cosf(start_alpha);

    start_pos_x = (int)((-1.f*b1 + 1.f*b2 )/(1.f*a1 - 1.f*a2));
    start_pos_y = a1*start_pos_x+b1;

// twierdzenie cosinusow, rownanie prostej prostopadlej, trygonometria, długosc odcinka
    int world_pos_x = (0 - start_pos_x)-sqrtf(2)*sqrtf((powf(start_pos_x,2)+powf(start_pos_y,2))*(1-cosf(180+start_alpha)))*cosf(atanf(-start_pos_x/start_pos_y));
    int world_pos_y = (0 - start_pos_y)-sqrtf(2)*sqrtf((powf(start_pos_x,2)+powf(start_pos_y,2))*(1-cosf(180+start_alpha)))*sinf(atanf(-start_pos_x/start_pos_y));
/*
Wyprowadzony wzór na przekszałcenie przy układach współrzędnych,
z układu robota(x,y) na układ świata(px,py)
przy zmianie pozycji robota o r i kąta o alpha
// px = sqrt(x^2 + y^2) * cos(arctg(y/x) - alpha) - r * cos(alpha)
// py = sqrt(x^2 + y^2) * sin(arctg(y/x) - alpha) - r * sin(alpha)
*/
    *start_x = world_pos_x;
    *start_y = world_pos_y;
}

// used to check data from sensors before appending it in sensor_data table
boolean check_if_measurements_ok(int *sensor)
{
    for(int i = 0; i < N_SENSORS; i++)
    {
        if(sensor[i] > 2000 || sensor[i] < 1)
            return FALSE;
    }

    return TRUE;
}
/*
 * robot axis is (0,0)
 * measurements from left to right
 **/
//     P1 = (-d1, 0) + (-310, 230)
//     P2 = (-d2 * sqrt(2) / 2, d2 * sqrt(2) / 2) + (-180, 330)
//     P3 = (0, d3) + (0, 380)
//     P4 = (d4 * sqrt(2) / 2, d4 * sqrt(2) / 2) + (180, 330)
//     P5 = (d5, 0) + (310, 230)
// const values below inherit from measurements on real robot
void transform_from_sensors_to_robot_coordinates(int *sensor, int *data_x, int *data_y){
    data_x[0] = -sensor[0] - 310;
    data_y[0] = 230;
    data_x[1] = -sensor[1]*sqrt(2)/2 - 180;
    data_y[1] = sensor[1]*sqrt(2)/2 + 330;
    data_x[2] = 0;
    data_y[2] = sensor[2] + 380;
    data_x[3] = sensor[3]*sqrt(2)/2 + 180;
    data_y[3] = sensor[3]*sqrt(2)/2 + 330;
    data_x[4] = sensor[4] + 310;
    data_y[4] = 230;
}

void transform_from_robot_to_world_coordinates(int *data_x, int *data_y, int data_size){
	for(int i = 0; i < data_size; i++)
	{
		float x = (float)data_x[i], y = (float)data_y[i];
		float a = __alpha - PI/2;
		data_x[i] = (int)(__x + x*cosf(a) - y*sinf(a));
		data_y[i] = (int)(__y + x*sinf(a) + y*cosf(a));
	}
}

int cramer(int x1, int y1, int x2, int y2, int *a, int *b){
    float W = (x1 - x2) *1.f;
    if(W < 0.0001)
        return 1;
    float Wa = (y1 - y2) *1.f;
    float Wb = (x1*y2 - x2*y1) *1.f;
    *a = (int)(Wa/W);
    *b = (int)(Wb/W);
    return 0;
}

void append_one_cycle_points(int *data_x, int *data_y, int data_size){
    for(int i = 0; i < data_size; i++)
    {
        x_buffer[counter] = data_x[i];
        y_buffer[counter] = data_y[i];
        counter = (counter + 1) % N_WALL_POINTS;
    }
}

void angle_sort_of_points(){

}

// earlier code is not used

/* zakładamy że stoimy w punkcie (90,90) i kącie 270 degrees  */


// px = sqrt(x^2 + y^2) * cos(arctg(y/x) - alpha) - r * cos(alpha)
// py = sqrt(x^2 + y^2) * sin(arctg(y/x) - alpha) - r * sin(alpha)

// cyclic function 30 Hz
void slam_walls_detection()
{
	for(int i = 1; i <= N_SENSORS; i++)
	{
		int d = dist(i);
		if(d > 2000 || d < 1){}
		else
			dist_meas[i - 1] = d;
	}
	transform_from_sensors_to_robot_coordinates(dist_meas, x_data, y_data);
	transform_from_robot_to_world_coordinates(x_data, y_data, N_SENSORS);
	append_one_cycle_points(x_data, y_data, N_SENSORS);


	/*
	 * TODO chcemy obliczyć za pomocą regresji liniowej i aproksymacji, proste opisujące ścianki
	 * korelujemy z realnymi znanymi równaniami prostych, aktualizujemy pozycję i kąt
	 * jeśli różnica pomiędzy poprzednim wyniesie więcej niż jedną kratkę wyrzuc Error()
	 * */



}
