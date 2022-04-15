#include <iostream>
#include <iomanip>
#include <random>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <fstream>
#include <cctype>
#include <chrono>
#define G 6.674e-11
using namespace std;
using std::vector;
/* Creamos la estructura con los datos a guardar */
struct space_aos{
    double vec_x;
    double vec_y;
    double vec_z;
    double vec_m;
    double vec_speed_x;
    double vec_speed_y;
    double vec_speed_z;
    double time_step;
};

/* Generamos el espacio */
void generate_space(vector <space_aos> &space, int random_seed, int num_objects, double size_enclosure,double time_step){
    ofstream file_init;
    /* Creamos las distribuciones */
    std::mt19937_64 gen (random_seed);
    std::uniform_real_distribution<double> dis(0.0, size_enclosure);
    std::normal_distribution<double> distribution(1e21,1e15);
    /* Abrimos el archivo para escribir la config inicial*/
    file_init.open("init_config.txt",ios::out);
    /*si da error cerramos el programa*/
    if(file_init.fail()){
        cerr << "No se pude abrir el archivo" << endl;
        exit(1);
    }

    file_init << fixed<<setprecision(3) << size_enclosure << " " << fixed<<setprecision(3) << time_step << " " <<
              fixed<<setprecision(3) <<num_objects << endl;
    /* Creamos los objetos y los vamos metiendo en el vector*/
    struct space_aos space_struct;
    for (int i=0; i<num_objects; i++){
        space_struct.vec_x= dis(gen);
        space_struct.vec_y=dis(gen);
        space_struct.vec_z=dis(gen);
        space_struct.vec_m=distribution(gen);
        space_struct.vec_speed_x= 0;
        space_struct.vec_speed_y= 0;
        space_struct.vec_speed_z= 0;
        space_struct.time_step = time_step;
        file_init << space_struct.vec_x << " " << space_struct.vec_y << " " << space_struct.vec_z
                  << " " << space_struct.vec_speed_x << " " << space_struct.vec_speed_y << " " << space_struct.vec_speed_z
                  << " " << space_struct.vec_m << endl;
        space.push_back(space_struct);
    }
    file_init.close();

}

void calculate_collision(vector <space_aos> &space , int obj_1, int obj_2, int &num_objects_final) {
    /* Se suman las masas */
    space[obj_1].vec_m = space[obj_1].vec_m + space[obj_2].vec_m;
    /* Se suman las velocidades */
    space[obj_1].vec_speed_x = space[obj_1].vec_speed_x + space[obj_2].vec_speed_x;
    space[obj_1].vec_speed_y = space[obj_1].vec_speed_y + space[obj_2].vec_speed_y;
    space[obj_1].vec_speed_z = space[obj_1].vec_speed_z + space[obj_2].vec_speed_z;
    /* La masa a 0 para "señalar" que está desaparecido */
    space[obj_2].vec_m=0;
    /* Restamos 1 al número total de objetos */
    num_objects_final -= 1;
}

void calculate_collisions(vector <space_aos> &space,int num_objects, int &num_objects_final){
    /* Un vector que nos ayuda con las distancias */
    double vector[3];
    /* Recorremos los objetos */
    for (int i = 0; i < num_objects; i++) {
        /* Si no han colisionado */
        if(0 != space[i].vec_m) {
            /* Calculamos la distancia con el resto de objetos */
            for (int j = 0; j < num_objects; j++) {
                /* Si son el mismo ahorramos cálculos */
                if (i != j) {
                    /* Si el otro no ha colisionado */
                    if (0 != space[j].vec_m) {
                        /* Vector para las distancias */
                        vector[0] = space[j].vec_x - space[i].vec_x;
                        vector[1] = space[j].vec_y - space[i].vec_y;
                        vector[2] = space[j].vec_z - space[i].vec_z;
                        /*distancia*/
                        double norma = std::sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
                        if (1 > norma){
                            calculate_collision(space, i, j, num_objects_final);
                        }
                    }
                }
            }
        }
    }
}


/*juntamos aceleracion y velocida en la misma funcion*/
void calculate_acceleration(double fuerza[3], int i,vector <space_aos> &space) {
    double time = space[i].time_step;
    double mass = space[i].vec_m;
    space[i].vec_speed_x += (fuerza[0]/mass) * time;
    space[i].vec_speed_y += (fuerza[1]/mass) * time;
    space[i].vec_speed_z += (fuerza[2]/mass) * time;
}


void calculate_forces(vector <space_aos> &space, int num_objects) {
    /* Un vector que almacena las fuerzas */
    double fuerza[3];
    /* Un vector que nos ayuda con las distancias */
    double vector[3];
    /* Recorremos los objetos */
    for (int i = 0; i < num_objects; i++) {
        /* Si no han colisionado */
        if(0 != space[i].vec_m) {
            /* Reiniciamos la fuerza para no usar la de la iteración anterior */
            fuerza[0] = 0;
            fuerza[1] = 0;
            fuerza[2] = 0;
            /* Calculamos la fuerza con el resto de objetos */
            for (int j = 0; j < num_objects; j++) {
                /* Si son el mismo ahorramos cálculos */
                if (i != j) {
                    /* Si el otro no ha colisionado */
                    if (0 != space[j].vec_m) {
                        /* Vector para las distancias */
                        vector[0] = space[j].vec_x - space[i].vec_x;
                        vector[1] = space[j].vec_y - space[i].vec_y;
                        vector[2] = space[j].vec_z - space[i].vec_z;
                        /*distancia*/
                        double norma = std::sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
                        double modulo = (G * space[i].vec_m * space[j].vec_m) / (norma * norma);
                        fuerza[0] += modulo * (vector[0] / norma);
                        fuerza[1] += modulo * (vector[1] / norma);
                        fuerza[2] += modulo * (vector[2] / norma);
                    }
                }
            }
            calculate_acceleration(fuerza, i, space);
        }
    }
}

void calculate_position(vector <space_aos> &space, int i){
    /*actualizamos posicion*/
    double time = space[i].time_step;
    space[i].vec_x+=space[i].vec_speed_x*time;
    space[i].vec_y+=space[i].vec_speed_y*time;
    space[i].vec_z+=space[i].vec_speed_z*time;
}

void calculate_bouncing(vector <space_aos> &space, int i, double size_enclosure){
    /* Calculamos rebotes */
    if (0 >= space[i].vec_x){
        space[i].vec_x=0;
        space[i].vec_speed_x*=-1;
    }

    if (0 >= space[i].vec_y){
        space[i].vec_y=0;
        space[i].vec_speed_y*=-1;
    }

    if (0 >= space[i].vec_z){
        space[i].vec_z=0;
        space[i].vec_speed_z*=-1;
    }

    if (size_enclosure <= space[i].vec_x){
        space[i].vec_x=size_enclosure;
        space[i].vec_speed_x*=-1;
    }

    if (size_enclosure <= space[i].vec_y){
        space[i].vec_y=size_enclosure;
        space[i].vec_speed_y*=-1;
    }

    if (size_enclosure <= space[i].vec_z){
        space[i].vec_z=size_enclosure;
        space[i].vec_speed_z*=-1;
    }
}

int main(int argc, char *argv[]) {
    using namespace std;
    ofstream file_final;
    char *param[5];
    int error = 0;
    for (int i = 1; i < argc; i++){
        param[i-1] = argv[i];
    }
    for (int i = argc-1; i<5;i++){
        param[i] = (char*)"?";
    }
    if (6 > argc){
        error = 1;
        cerr << "Error:wrong number of parametres"<< endl;
    }
    else if (0 >= atoi(param[0])){
        cerr << "Error: invalid number of objects"<< endl;
        error = 2;
    }

    else if (0 >= atoi(param[1])){
        cerr << "Error: invalid number of iterations" << endl;
        error = 2;
    }

    else if (0 >= atoi(param[2])){
        cerr << "Error: invalid random seed" << endl;
        error = 2;
    }

    else if (0 >= atof(param[3])){
        cerr << "Error: invalid size enclosure" << endl;
        error = 2;
    }

    else if (0 >= atof(param[4])){
        cerr << "Error: invalid time_Step" << endl;
        error = 2;
    }
    else{
        cout << "creating simulation"<< endl;
        cout << "  num_objects: "<< param[0] << endl;
        cout << "  num_iterations: "<< param[1] << endl;
        cout << "  random_seed: "<< param[2]<< endl;
        cout << "  size_enclosure: "<< param[3] << endl;
        cout << "  time_step: "<< param[4] << endl;}
    if (0 < error) {
        cerr << "sim-soa invoked with "<< argc-1 << " parameters."<< endl;
        cerr << "Arguments" << endl;
        cerr << "  num_objects: "<< param[0] << endl;
        cerr << "  num_iterations: "<< param[1] << endl;
        cerr << "  random_seed: "<< param[2] << endl;
        cerr << "  size_enclosure: "<< param[3] << endl;
        cerr << "  time_step: "<< param[4] << endl;
        if (1 == error){
            exit(-1);
        }
        else{exit(-2);}
    }

    /* Sacamos los parametros */
    int num_objects = atoi(argv[1]);
    int num_iterations = atoi(argv[2]);
    int random_seed = atoi(argv[3]);
    double size_enclosure = atof(argv[4]);
    double time_step = atof(argv[5]);
    int num_objects_final = num_objects;


    /*creamos el vector que contendra todas las structs */
    vector <struct space_aos> space;
    /* Generamos el espacio */
    generate_space(space, random_seed, num_objects, size_enclosure,time_step);
    using namespace std::chrono;
    using clk = chrono::high_resolution_clock;
    for (int i=0; i<num_iterations; i++){
        auto t1 = clk::now();
        /* Calculamos las fuerzas en general */
        calculate_forces(space, num_objects);
        for (int j=0; j<num_objects; j++) {
            /* Si no han colisionado calculamos las posiciones */
            if (0 != space[j].vec_m ){
                calculate_position(space, j);
                calculate_bouncing(space, j, size_enclosure);
            }
        }
        /* Calculamos las colisiones */
        calculate_collisions(space, num_objects,num_objects_final);
        auto t2 = clk::now();
        auto diff = duration_cast<microseconds>(t2-t1);
        cout<<diff.count()<<"\n";
    }
    /* Escribimos el final_config */
    file_final.open("final_config.txt",ios::out);

    if(file_final.fail()){
        cerr << "No se pude abrir el archivo" << endl;
        exit(1);
    }
    file_final << fixed<<setprecision(3) << size_enclosure << " " << fixed<<setprecision(3) << time_step << " " <<
               fixed<<setprecision(3) <<num_objects_final << endl;
    for (int i=0; i<num_objects; i++) {
        if (space[i].vec_m != 0){
            file_final<< space[i].vec_x << " " << space[i].vec_y << " " << space[i].vec_z << " " << space[i].vec_speed_x <<
                      " " << space[i].vec_speed_y << " " << space[i].vec_speed_z << " " << space[i].vec_m << endl;
        }
    }
    file_final.close();
}

