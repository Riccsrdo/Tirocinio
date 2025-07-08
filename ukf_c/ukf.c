/*
Per compilare, serve linkare con gcc la libreria GSL
e includere le opzioni -lm -lgsl -lgslcblas.

*/

#include"ukf.h"
#include <gsl/gls_linalg.h>
#include <gsl/gsl_blas.h>
#include<math.h>

// Funzione interna che calcola i punti sigma
static void generate_sigma_points(ukf_state *state, double lambda){
    const int n = state->dim_x; // prende la dimensione dello stato
    const double alpha_sq_lambda = sqrt(n+lambda); 

    // Copio P in una matrice temporanea per la decomposizione di Cholesky
    gsl_matrix_memcpy(state->tmp_P, state->P);

    // Calcolo la decomposizione di Cholesky
    // L * L^T = P
    if(gsl_linalg_cholesky_decomp(state->tmp_P) != GLS_SUCCESS) {
        // in caso di errore, setto matrice identità
        gsl_matrix_set_identity(state->tmp_P);
    }

    // Genero i punti sigma
    // il primo è dato dallo stato attuale
    gsl_matrix_set_col(state->sigma_points, 0, state->x);

    // Calcolo gli altri
    for(int i = 0; i<n; ++i){
        gsl_vector_view col_i = gsl_matrix_column(state->tmp_P, i);

        // x + sqrt(n+lambda)*L_i (matrice colonna i-esima di L)
        gsl_vector_memcpy(state->tmp_x, state->x);
        gsl_blas_daxpy(alpha_sq_lambda, &col_i.vector, state->tmp_x); // tmp_x = x + sqrt(n+lambda)*L_i
        gsl_matrix_set_col(state->sigma_points, i+1, state->tmp_x); // setto colonna i+1 della matrice sigma_points

        // x - sqrt(n+lambda)*L_i
        gsl_vector_memcpy(state->tmp_x, state->x);
        gsl_blas_daxpy(-alpha_sq_lambda, &col.vector, state->tmp_x);
        gsl_matrix_set_col(state->sigmas_f, i + 1 + n, state->tmp_x);
    }
}

// funzione di init
ukf_state* ukf_init(int dim_x, int dim_z, double alpha, double beta, double kappa,
                    void (*fx)(const gsl_vector*, double, gsl_vector*),
                    void (*hx)(const gsl_vector*, gsl_vector*)) {

    
    ukf_state *state = (ukf_state*) malloc(sizeof(ukf_state));
    if(!state) {
        return NULL; // errore allocazione memoria
    }

    state->dim_x = dim_x;
    state->dim_z = dim_z;
    state->num_sigma_points = 2 * dim_x + 1;
    state->fx = fx;
    state->hx = hx;

}