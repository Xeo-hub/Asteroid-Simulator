#include <iostream>
#include <random>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iomanip>
#define G 6.674e-11;
using std::fstream;
using std::vector;
/* Creamos la estructura con los datos a guardar */
struct space_soa{
    vector <double> vec_x;
    vector <double> vec_y;
    vector <double> vec_z;
    vector <double> vec_m;
    vector <double> vec_speed_x;
    vector <double> vec_speed_y;
    vector <double> vec_speed_z;
    int num_objects_final;
    double time_step;
};

/* Generamos el espacio */
void generate_space(struct space_soa &space, int random_seed, int num_objects, double size_enclosure) {
    /* Creamos las distribuciones */
    std::mt19937_64 gen(random_seed);
    std::uniform_real_distribution<double> dis(0.0, size_enclosure);
    std::normal_distribution<double> distribution(1e21,1e15);
    /* Creamos los objetos */
    for (int i = 0; i < num_objects; i++) {
        space.vec_x.push_back(dis(gen));
        space.vec_y.push_back(dis(gen));
        space.vec_z.push_back(dis(gen));
        space.vec_m.push_back(distribution(gen));
    }

    /* Por localidad espacial y estructura del soa separamos los 3 bucles */
    for (int i = 0; i < num_objects; i++) {
        space.vec_speed_x.push_back(0);
    }

    for (int i = 0; i < num_objects; i++) {
        space.vec_speed_y.push_back(0);
    }

    for (int i = 0; i < num_objects; i++) {
        space.vec_speed_z.push_back(0);
    }
}

void calculate_collision(struct space_soa &space , int obj_1, int obj_2) {
    /* Se suman las masas */
    space.vec_m[obj_1] = space.vec_m[obj_1] + space.vec_m[obj_2];
    /* Se suman las velocidades */
    space.vec_speed_x[obj_1] = space.vec_speed_x[obj_1] + space.vec_speed_x[obj_2];
    space.vec_speed_y[obj_1] = space.vec_speed_y[obj_1] + space.vec_speed_y[obj_2];
    space.vec_speed_z[obj_1] = space.vec_speed_z[obj_1] + space.vec_speed_z[obj_2];
    /* La masa a 0 para "señalar" que está desaparecido */
    space.vec_m[obj_2]=0;
    /* Restamos 1 al número total de objetos */
    space.num_objects_final -=1;
}

void calculate_speed(double aceleracion[3], int i, struct space_soa &space) {
    /* Actualizamos las velocidades */
    double time = space.time_step;
    space.vec_speed_x[i] += aceleracion[0] * time;
    space.vec_speed_y[i] += aceleracion[1] * time;
    space.vec_speed_z[i] += aceleracion[2] * time;
}

void calculate_acceleration(double fuerza[3], int i, struct space_soa &space){
    /* Calculamos las aceleraciones */
    double aceleracion[3];
    double mass = space.vec_m[i];
    aceleracion[0] = (fuerza[0]/mass);
    aceleracion[1] = (fuerza[1]/mass);
    aceleracion[2] = (fuerza[2]/mass);
    calculate_speed(aceleracion, i, space);
}

void calculate_forces(struct space_soa &space, int num_objects, bool final) {
    /* Un vector que almacena las fuerzas */
    double fuerza[3];
    /* Un vector que nos ayuda con las distancias */
    double aux_vector[3];
    /* Recorremos los objetos */
    for (int i = 0; i < num_objects; i++) {
        /* Si no han colisionado */
        if (0!=space.vec_m[i]) {
            /* Reiniciamos la fuerza para no usar la de la iteración anterior */
            double masa = space.vec_m[i];
            fuerza[0] = 0;
            fuerza[1] = 0;
            fuerza[2] = 0;
            /* Calculamos la fuerza con el resto de objetos */
            for (int j = 0; j < num_objects; j++) {
                /* Si son el mismo ahorramos cálculos */
                if (i != j) {
                    /* Si el otro no ha colisionado */
                    if (0!=space.vec_m[j]) {
                        /* Vector para las distancias */
                        aux_vector[0] = space.vec_x[j] - space.vec_x[i];
                        aux_vector[1] = space.vec_y[j] - space.vec_y[i];
                        aux_vector[2] = space.vec_z[j] - space.vec_z[i];
                        /* Distancia */
                        double norma = std::sqrt(aux_vector[0] * aux_vector[0] + aux_vector[1] * aux_vector[1] +
                                                 aux_vector[2] * aux_vector[2]);
                        /* Si la distancia es menor que 1 colisionan */
                        if (1>norma) {
                            calculate_collision(space, i, j);
                        } else {
                            /* Un bool para que al final no haga cálculos innecesarios */
                            if (1 ==final) {
                                double var = masa * space.vec_m[j];
                                var = var / (norma * norma);
                                var = var * G;
                                fuerza[0] += var * (aux_vector[0] / norma);
                                fuerza[1] += var * (aux_vector[1] / norma);
                                fuerza[2] += var * (aux_vector[2] / norma);
                            }
                        }
                    }
                }
            }
            /* Un bool para no actualizar mal los datos al final */
            if (1==final) {
                calculate_acceleration(fuerza, i, space);
            }
        }
    }
}

void calculate_position(int i, struct space_soa &space){
    /* Actualizamos posición */
    double time = space.time_step;
    space.vec_x[i]+=space.vec_speed_x[i]*time;
    space.vec_y[i]+=space.vec_speed_y[i]*time;
    space.vec_z[i]+=space.vec_speed_z[i]*time;
}

void calculate_bouncing(struct space_soa &space, int i, double size_enclosure){
    /* Calculamos rebotes */
    if (0>=space.vec_x[i]){
        space.vec_x[i]=0;
        space.vec_speed_x[i]*=-1;
    }
    if (0>=space.vec_y[i]){
        space.vec_y[i]=0;
        space.vec_speed_y[i]*=-1;
    }
    if (0>=space.vec_z[i]){
        space.vec_z[i]=0;
        space.vec_speed_z[i]*=-1;
    }
    if (size_enclosure<=space.vec_x[i]){
        space.vec_x[i]=size_enclosure;
        space.vec_speed_x[i]*=-1;
    }
    if (size_enclosure<=space.vec_y[i]){
        space.vec_y[i]=size_enclosure;
        space.vec_speed_y[i]*=-1;
    }
    if (size_enclosure<=space.vec_z[i]){
        space.vec_z[i]=size_enclosure;
        space.vec_speed_z[i]*=-1;
    }
}

int main(int argc, char *argv[]) {
    using namespace std;

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
    /* Creamos las estructuras y ficheros */
    struct space_soa space;
    space.num_objects_final = num_objects;
    space.time_step = time_step;
    ofstream outfile_initial;
    ofstream outfile_final;
    /* Metemos los datos iniciales */
    outfile_initial.open("init_config.txt", ios::out);
    if (!outfile_initial){cerr<<"Error"; exit(-1);}
    else {
        outfile_initial<< fixed << setprecision(3)<<size_enclosure<<" "<<space.time_step<<" "<<num_objects<<endl;
        /* Generamos el espacio */
        generate_space(space, random_seed, num_objects, size_enclosure);
        for (int i=0; i<num_objects; i++){
            outfile_initial<< fixed << setprecision(3) << space.vec_x[i]<<" "<<space.vec_y[i]<<" "<<space.vec_z[i]
                           <<" "<<space.vec_speed_x[i]<<" "<<space.vec_speed_y[i]<<" "<<space.vec_speed_z[i]<<" "<<space.vec_m[i]<< endl;
        }
        /* Cerramos fichero */
        outfile_initial.close();

        /* Empieza */
        int final = 1;
        for (int i = 0; i < num_iterations; i++) {
            /* Calculamos las fuerzas en general */
            calculate_forces(space, num_objects, final);
            for (int j = 0; j < num_objects; j++) {
                /* Si no han colisionado calculamos las posiciones */
                if (0 != space.vec_m[j]) {
                    calculate_position(j, space);
                    calculate_bouncing(space, j, size_enclosure);
                }
            }
            /* Calculamos las colisiones al final de la ultima iteracion sin actualizar los datos */
            if (num_iterations == i + 1) {
                final = 0;
                calculate_forces(space, num_objects, final);
            }
        }
        /* Escribimos el final_config */
        outfile_final.open("final_config.txt", ios::out);
        if (!outfile_final){cout<<"Error";}
        else {
            outfile_final<< fixed << setprecision(3)<<size_enclosure<<" "<<space.time_step<<" "<<space.num_objects_final<<endl;
            for (int i = 0; i < num_objects; i++) {
                if (0 != space.vec_m[i]){
                    outfile_final << fixed << setprecision(3) << space.vec_x[i] << " " << space.vec_y[i] << " "<< space.vec_z[i]
                                  << " " << space.vec_speed_x[i] << " " << space.vec_speed_y[i] << " "
                                  << space.vec_speed_z[i] << " " << space.vec_m[i] << endl;

                }
            }
            outfile_final.close();
        }
    }
}

