//
// Created by Adrian on 10/12/2017.
//
#include "rgb-depth-sync/Segmentacion.h"

void Segmentacion::void mapear(vector<Punto3D*> &puntos, const TangoPointCloud *nube, int w, int h) {

    int point_cloud_size = nube->num_points;
    for (int i = 0; i < point_cloud_size; ++i) {
        float x = nube->points[i][0];
        float y = nube->points[i][1];
        float z = nube->points[i][2];

        // depth_t0_point is the point in depth camera frame on timestamp t0.
        // (depth image timestamp).
        glm::vec4 depth_t0_point = glm::vec4(x, y, z, 1.0);

        int pixel_x, pixel_y;
        // get the coordinate on image plane.
        pixel_x = static_cast<int>((rgb_camera_intrinsics_.fx) *
                                   (color_t1_point.x / color_t1_point.z) +
                                   rgb_camera_intrinsics_.cx);

        pixel_y = static_cast<int>((rgb_camera_intrinsics_.fy) *
                                   (color_t1_point.y / color_t1_point.z) +
                                   rgb_camera_intrinsics_.cy);

        // Color value is the GL_LUMINANCE value used for displaying the depth
        // image.
        // We can query for depth value in mm from grayscale image buffer by
        // getting a `pixel_value` at (pixel_x,pixel_y) and calculating
        // pixel_value * (kMaxDepthDistance / USHRT_MAX)
        float depth_value = color_t0_point.z;
        uint8_t grayscale_value =
                (color_t1_point.z * kMeterToMillimeter) * UCHAR_MAX / kMaxDepthDistance;

    }

}

int Segmentacion::procesar(vector<Punto3D *> &puntos, map<int, Plano3D> &planos, map<int, Elemento3D> &elementos, bool(*f)(Punto3D *, Punto3D *), bool(*f2)(Punto3D *, Punto3D *), int w, int h, string archivo, set<int> &suelo, set<int> &relevantes) {


    vector<float> integralX = vector<float>(w * h, 0);
    vector<float> integralY = vector<float>(w * h, 0);
    vector<float> integralZ = vector<float>(w * h, 0);
    calcularIntegralX(puntos, integralX, w, h);
    calcularIntegralY(puntos, integralY, w, h);
    calcularIntegralZ(puntos, integralZ, w, h);

    vector<int> vecinos = vector<int>(w * h, 0);

    calcularMapaDinamicoDeVecinos(puntos, vecinos, w, h);

    calcularNormales(puntos, w, h, integralX, integralY, integralZ, vecinos);

    vector<int> padres;
    aplicarConnectedComponentsLabeling(puntos, f, padres, w, h);

    vector<int> contadorPuntosEnPadres = vector<int>(padres.size(), 0);
    reducirPadres(puntos, padres, contadorPuntosEnPadres);

    asignarPuntosAPlanos(puntos, planos, contadorPuntosEnPadres);

    borrarPlanosNoValidos(planos);

    extenderPlanos(puntos, w, h);

    //recalcularParametrosDelPlano(planos);

    combinarPlanos(planos);

    vector<int> padresDepth;
    vector<int> etiquetasDepth = vector<int>(w * h, 0);
    aplicarConnectedComponentsLabelingDepth(puntos, f2, padresDepth, etiquetasDepth, w, h);

    vector<int> contadorPuntosEnPadresDepth = vector<int>(padresDepth.size(), 0);
    reducirPadresYAplicarEtiquetasDepth(puntos, padresDepth, etiquetasDepth, contadorPuntosEnPadresDepth);


    asignarPuntosAElementos(puntos, elementos, contadorPuntosEnPadresDepth);
    detectarSuelo(planos, suelo);
    detectarRelevantes(puntos, planos, elementos, suelo, relevantes);

    return relevantes.size();

}

void Segmentacion::calcularNormales(vector<Punto3D *> &puntos, int w, int h, vector<float> integralX, vector<float> integralY, vector<float> integralZ, vector<int> vecinos) {

    //int r = radioNormalIntegral;

    for (int i = 1; i < (w - 1); i++) {
        for (int j = 1; j < (h - 1); j++) {

            int r = vecinos[j * w + i];
            r = r < 2 ? 2 : r;

            if (puntos[j * w + i]->getValido() && r > 0) {

                Punto3D *punto = puntos[j * w + i];

                int ipr = (i + r < w) ? i + r : w - 1;
                int imr = (i - r >= 0) ? i - r : 0;
                int jpr = (j + r < h) ? j + r : h - 1;
                int jmr = (j - r >= 0) ? j - r : 0;
                int ip1 = (i + 1 < w) ? i + 1 : w - 1;
                int im1 = (i - 1 >= 0) ? i - 1 : 0;
                int jp1 = (j + 1 < h) ? j + 1 : h - 1;
                int jm1 = (j - 1 >= 0) ? j - 1 : 0;

                //float vhx = (puntos[j * w + (ipr)]->getX() - puntos[j * w + (imr)]->getX()) / 2;
                //float vhy = (puntos[j * w + (ipr)]->getY() - puntos[j * w + (imr)]->getY()) / 2;
                //float vhz = (puntos[j * w + (ipr)]->getZ() - puntos[j * w + (imr)]->getZ()) / 2;
                float vhx = (calcularSEnLaIntegral(integralX, w, h, ip1, j, r - 1) - calcularSEnLaIntegral(integralX, w, h, im1, j, r - 1));
                float vhy = (calcularSEnLaIntegral(integralY, w, h, ip1, j, r - 1) - calcularSEnLaIntegral(integralY, w, h, im1, j, r - 1));
                float vhz = (calcularSEnLaIntegral(integralZ, w, h, ip1, j, r - 1) - calcularSEnLaIntegral(integralZ, w, h, im1, j, r - 1));

                //float vvx = (puntos[jpr * w + (i)]->getX() - puntos[jmr * w + (i)]->getX()) / 2;
                //float vvy = (puntos[jpr * w + (i)]->getY() - puntos[jmr * w + (i)]->getY()) / 2;
                //float vvz = (puntos[jpr * w + (i)]->getZ() - puntos[jmr * w + (i)]->getZ()) / 2;
                float vvx = (calcularSEnLaIntegral(integralX, w, h, i, jp1, r - 1) - calcularSEnLaIntegral(integralX, w, h, i, jm1, r - 1));
                float vvy = (calcularSEnLaIntegral(integralY, w, h, i, jp1, r - 1) - calcularSEnLaIntegral(integralY, w, h, i, jm1, r - 1));
                float vvz = (calcularSEnLaIntegral(integralZ, w, h, i, jp1, r - 1) - calcularSEnLaIntegral(integralZ, w, h, i, jm1, r - 1));

                punto->setNX(vhy * vvz - vhz * vvy);
                punto->setNY(vhz * vvx - vhx * vvz);
                punto->setNZ(vhx * vvy - vhy * vvx);
                punto->normalizarVectorNormal();
                punto->calcularND();
                punto->setNormalValida(true);

            } else {
                puntos[j * w + i]->setNormalValida(false);

            }
        }
    }
}

void Segmentacion::calcularIntegralX(vector<Punto3D *> &puntos, vector<float> &integral, int w, int h) {

    //Primer elemento
    integral[0] = puntos[0]->getX();
    //Primera fila
    for (int i = 1; i < w; i++) {
        integral[i] = puntos[i]->getX() + integral[i - 1];
    }
    //Primera columna
    for (int j = 1; j < h; j++) {
        integral[j * w] = puntos[j * w]->getX() + integral[(j - 1) * w];
    }
    //El resto
    for (int i = 1; i < w; i++) {
        for (int j = 1; j < h; j++) {
            integral[(j * w) + i] = puntos[(j * w) + i]->getX() + integral[(j * w) + (i - 1)] + integral[(j - 1) * w + i] - integral[(j - 1) * w + (i - 1)];
        }
    }
}

void Segmentacion::calcularIntegralY(vector<Punto3D *> &puntos, vector<float> &integral, int w, int h) {

    //Primer elemento
    integral[0] = puntos[0]->getY();
    //Primera fila
    for (int i = 1; i < w; i++) {
        integral[i] = puntos[i]->getY() + integral[i - 1];
    }
    //Primera columna
    for (int j = 1; j < h; j++) {
        integral[j * w] = puntos[j * w]->getY() + integral[(j - 1) * w];
    }
    //El resto
    for (int i = 1; i < w; i++) {
        for (int j = 1; j < h; j++) {
            integral[(j * w) + i] = puntos[(j * w) + i]->getY() + integral[(j * w) + (i - 1)] + integral[(j - 1) * w + i] - integral[(j - 1) * w + (i - 1)];
        }
    }
}

void Segmentacion::calcularIntegralZ(vector<Punto3D *> &puntos, vector<float> &integral, int w, int h) {

    //Primer elemento
    integral[0] = puntos[0]->getZ();
    //Primera fila
    for (int i = 1; i < w; i++) {
        integral[i] = puntos[i]->getZ() + integral[i - 1];
    }
    //Primera columna
    for (int j = 1; j < h; j++) {
        integral[j * w] = puntos[j * w]->getZ() + integral[(j - 1) * w];
    }
    //El resto
    for (int i = 1; i < w; i++) {
        for (int j = 1; j < h; j++) {
            integral[(j * w) + i] = puntos[(j * w) + i]->getZ() + integral[(j * w) + (i - 1)] + integral[(j - 1) * w + i] - integral[(j - 1) * w + (i - 1)];
        }
    }
}

int Segmentacion::calcularMapaDinamicoDeVecinos(vector<Punto3D *> &puntos, vector<int> &vecinos, int w, int h) {
    //R(m,n) = min( B(m,n), T(m,n)/sqrt(2))
    vector<pair<int, int>> porProcesar;
    vector<int> distancias = vector<int>(w * h, -1);
    set<int> asignados;


    //no se aplica a los bordes
    for (int i = 1; i < (w - 1); i++) {
        for (int j = 1; j < (h - 1); j++) {

            if (puntos[j * w + i]->getValido()) {
                //Calculamos B
                int b = sumaBeta + (round(beta * (alfa * puntos[j * w + i]->getDepth() * puntos[j * w + i]->getDepth())));
                vecinos[j * w + i] = b >= 1 ? b : 1;

                //Comprobamos si es un punto para empezar el cálculo de C
                float threshold = sumaDelta + delta * (alfa * puntos[j * w + i]->getDepth() * puntos[j * w + i]->getDepth());

                if (abs(puntos[j * w + i]->getDepth() - puntos[j * w + (i + 1)]->getDepth()) >= threshold
                    || abs(puntos[j * w + i]->getDepth() - puntos[(j + 1) * w + i]->getDepth()) >= threshold) {
                    porProcesar.push_back(pair<int, int>(i, j));
                    distancias[j * w + i] = 0;
                    asignados.insert(j * w + i);
                }
            }
        }
    }
    //Calculamos las distancias para T
    for (int t = 0; t < porProcesar.size(); t++) {

        int i = porProcesar[t].first;
        int j = porProcesar[t].second;
        int d = distancias[j * w + i];

        //Izquierda
        if (i - 1 >= 0 && distancias[j * w + (i - 1)] == -1 && asignados.count(j * w + (i - 1)) == 0) {

            porProcesar.push_back(pair<int, int>(i - 1, j));
            distancias[j * w + (i - 1)] = d + 1;
            asignados.insert(j * w + (i - 1));
        }

        //Derecha
        if (i + 1 < w && distancias[j * w + (i + 1)] == -1 && asignados.count(j * w + (i + 1)) == 0) {

            porProcesar.push_back(pair<int, int>(i + 1, j));
            distancias[j * w + (i + 1)] = d + 1;
            asignados.insert(j * w + (i + 1));
        }

        //Arriba
        if (j - 1 >= 0 && distancias[(j - 1) * w + i] == -1 && asignados.count((j - 1) * w + i) == 0) {

            porProcesar.push_back(pair<int, int>(i, (j - 1)));
            distancias[(j - 1) * w + i] = d + 1;
            asignados.insert((j - 1) * w + i);
        }

        //Abajo
        if (j + 1 < h && distancias[(j + 1) * w + i] == -1 && asignados.count((j + 1) * w + i) == 0) {

            porProcesar.push_back(pair<int, int>(i, (j + 1)));
            distancias[(j + 1) * w + i] = d + 1;
            asignados.insert((j + 1) * w + i);
        }

    }

    //Calcular el minimo entre B y T
    float raiz2 = sqrt(2);
    int maxVecinos = -1;

    for (int i = 0; i < w * h; i++) {

        if (puntos[i]->getValido() && distancias[i] >= 0) {

            int t = round(distancias[i] / raiz2);
            if (t < vecinos[i])
                vecinos[i] = t;

        }

        if (maxVecinos < vecinos[i]) maxVecinos = vecinos[i];

    }

    return maxVecinos;

}

float Segmentacion::calcularSEnLaIntegral(vector<float> &integral, int w, int h, int m, int n, int r) {

    //Si el radio es 0(solo el punto), la media es el propio valor;
    if (r == 0) {

        return integral[n * w + m];

    } else {
        int mpr = (m + r < w) ? m + r : w - 1;
        int mmr = (m - r >= 0) ? m - r : 0;
        int npr = (n + r < h) ? n + r : h - 1;
        int nmr = (n - r >= 0) ? n - r : 0;
        int div = 4 * r * r; //(r * 2 + 1)*(r * 2 + 1);

        return (integral[((npr) * w) + (mpr)]
                - integral[((npr) * w) + (mmr)]
                - integral[((nmr) * w) + (mpr)]
                + integral[((nmr) * w) + (mmr)])
               / div;
    }
}

int Segmentacion::aplicarConnectedComponentsLabeling(vector<Punto3D *> &puntos, bool(*f)(Punto3D *, Punto3D *), vector<int> &padres, int w, int h) {

    int contador = 1;

    //Para que padres empiece en 1
    padres.push_back(0);

    //Primer pixel
    if (puntos[0]->getNormalValida()) {
        puntos[0]->setEtiqueta(contador++);
        padres.push_back(0);
    }
    //Primera fila
    for (int i = 1; i < w; i++) {
        if (puntos[i]->getNormalValida()) {
            if (puntos[i - 1]->getNormalValida() && (*f)(puntos[i - 1], puntos[i])) {
                puntos[i]->setEtiqueta(puntos[i - 1]->getEtiqueta());
            } else {
                puntos[i]->setEtiqueta(contador++);
                padres.push_back(0);
            }
        }
    }
    //Primera columna
    for (int j = 1; j < h; j++) {
        if (puntos[j * w]->getNormalValida()) {
            if (puntos[(j - 1) * w]->getNormalValida() && (*f)(puntos[(j - 1) * w], puntos[j * w])) {
                puntos[j * w]->setEtiqueta(puntos[(j - 1) * w]->getEtiqueta());
            } else {
                puntos[j * w]->setEtiqueta(contador++);
                padres.push_back(0);
            }
        }
    }
    //El resto
    for (int i = 1; i < w; i++) {
        for (int j = 1; j < h; j++) {

            if (puntos[(j * w) + i]->getNormalValida()) {

                bool igualArriba = false;
                bool igualIzquierda = false;

                if (puntos[(j * w) + (i - 1)]->getNormalValida() && (*f)(puntos[(j * w) + (i - 1)], puntos[(j * w) + i])) {
                    igualIzquierda = true;
                }
                if (puntos[((j - 1) * w) + i]->getNormalValida() && (*f)(puntos[((j - 1) * w) + i], puntos[(j * w) + i])) {
                    igualArriba = true;
                }

                if (igualArriba && igualIzquierda) {
                    unionEtiquetas(puntos[((j - 1) * w) + i]->getEtiqueta(), puntos[(j * w) + (i - 1)]->getEtiqueta(), padres);
                    puntos[(j * w) + i]->setEtiqueta(puntos[(j * w) + (i - 1)]->getEtiqueta());
                } else if (igualArriba) {
                    puntos[(j * w) + i]->setEtiqueta(puntos[((j - 1) * w) + i]->getEtiqueta());
                } else if (igualIzquierda) {
                    puntos[(j * w) + i]->setEtiqueta(puntos[(j * w) + (i - 1)]->getEtiqueta());
                } else {
                    puntos[(j * w) + i]->setEtiqueta(contador++);
                    padres.push_back(0);
                }
            }
        }
    }

    return contador;

}

int Segmentacion::aplicarConnectedComponentsLabelingDepth(vector<Punto3D *> &puntos, bool(*f)(Punto3D *, Punto3D *), vector<int> &padres, vector<int> &etiquetas, int w, int h) {

    int contador = 1;

    //Para que padres empiece en 1
    padres.push_back(0);

    //Primer pixel
    if (puntos[0]->getEtiqueta() == 0 && puntos[0]->getValido()) {
        etiquetas[0] = contador++;;
        padres.push_back(0);
    }
    //Primera fila
    for (int i = 1; i < w; i++) {
        if (puntos[i]->getEtiqueta() == 0 && puntos[i]->getValido()) {
            if (puntos[i - 1]->getEtiqueta() == 0 && puntos[i - 1]->getValido() && (*f)(puntos[i - 1], puntos[i])) {
                etiquetas[i] = etiquetas[i - 1];
            } else {
                etiquetas[i] = contador++;
                padres.push_back(0);
            }
        }
    }
    //Primera columna
    for (int j = 1; j < h; j++) {
        if (puntos[j * w]->getEtiqueta() == 0 && puntos[j * w]->getValido()) {
            if (puntos[(j - 1) * w]->getEtiqueta() == 0 && puntos[(j - 1) * w]->getValido() && (*f)(puntos[(j - 1) * w], puntos[j * w])) {
                etiquetas[j * w] = etiquetas[(j - 1) * w];
            } else {
                etiquetas[j * w] = contador++;
                padres.push_back(0);
            }
        }
    }
    //El resto arriba izquierda
    for (int i = 1; i < w; i++) {
        for (int j = 1; j < h; j++) {

            if (puntos[(j * w) + i]->getEtiqueta() == 0) {

                bool igualArriba = false;
                bool igualIzquierda = false;

                if (puntos[(j * w) + (i - 1)]->getEtiqueta() == 0 && puntos[(j * w) + (i - 1)]->getValido() && (*f)(puntos[(j * w) + (i - 1)], puntos[(j * w) + i])) {
                    igualIzquierda = true;
                }
                if (puntos[((j - 1) * w) + i]->getEtiqueta() == 0 && puntos[((j - 1) * w) + i]->getValido() && (*f)(puntos[((j - 1) * w) + i], puntos[(j * w) + i])) {
                    igualArriba = true;
                }

                if (igualArriba && igualIzquierda) {
                    unionEtiquetas(etiquetas[((j - 1) * w) + i], etiquetas[(j * w) + (i - 1)], padres);
                    etiquetas[(j * w) + i] = etiquetas[(j * w) + (i - 1)];
                } else if (igualArriba) {
                    etiquetas[(j * w) + i] = etiquetas[((j - 1) * w) + i];
                } else if (igualIzquierda) {
                    etiquetas[(j * w) + i] = etiquetas[(j * w) + (i - 1)];
                } else {
                    etiquetas[(j * w) + i] = contador++;
                    padres.push_back(0);
                }
            }
        }
    }

    //El resto abajo derecha
    for (int i = w - 2; i >= 0; i--) {
        for (int j = h - 2; j >= 0; j--) {

            if (puntos[(j * w) + i]->getEtiqueta() == 0) {

                bool igualAbajo = false;
                bool igualDerecha = false;

                if (puntos[(j * w) + (i + 1)]->getEtiqueta() == 0 && puntos[(j * w) + (i + 1)]->getValido() && (*f)(puntos[(j * w) + (i + 1)], puntos[(j * w) + i])) {
                    igualDerecha = true;
                }
                if (puntos[((j + 1) * w) + i]->getEtiqueta() == 0 && puntos[((j + 1) * w) + i]->getValido() && (*f)(puntos[((j + 1) * w) + i], puntos[(j * w) + i])) {
                    igualAbajo = true;
                }

                if (igualAbajo && igualDerecha) {
                    unionEtiquetas(etiquetas[((j + 1) * w) + i], etiquetas[(j * w) + (i + 1)], padres);
                    etiquetas[(j * w) + i] = etiquetas[(j * w) + (i + 1)];
                } else if (igualAbajo) {
                    etiquetas[(j * w) + i] = etiquetas[((j + 1) * w) + i];
                } else if (igualDerecha) {
                    etiquetas[(j * w) + i] = etiquetas[(j * w) + (i + 1)];
                } else {
                    etiquetas[(j * w) + i] = contador++;
                    padres.push_back(0);
                }
            }
        }
    }

    return contador;

}

void Segmentacion::unionEtiquetas(int et1, int et2, vector<int> &padres) {

    while (padres[et1] != 0) et1 = padres[et1];
    while (padres[et2] != 0) et2 = padres[et2];
    if (et1 != et2) padres[et2] = et1;

}

int Segmentacion::reducirPadres(vector<Punto3D *> &puntos, vector<int> &padres, vector<int> &contadorPuntos) {

    //Calculamos el padre raiz de cada etiqueta
    for (int i = 1; i < padres.size(); i++) {
        if (padres[i] != 0) { // Si es raiz no hago nada
            int et = i;
            while (padres[et] != 0) et = padres[et];
            padres[i] = et;
        }
    }

    //Sustituimos la etiqueta por la raiz padre de todos los puntos
    for (int i = 0; i < puntos.size(); i++) {

        if (puntos[i]->getValido() && padres[puntos[i]->getEtiqueta()] != 0) {
            puntos[i]->setEtiqueta(padres[puntos[i]->getEtiqueta()]);
        }

        contadorPuntos[puntos[i]->getEtiqueta()] = contadorPuntos[puntos[i]->getEtiqueta()] + 1;

    }

    //Marcamos como etiqueta 0 a todos las etiquetas que tengan menos de minPuntos
    for (int i = 1; i < puntos.size(); i++) {
        if (puntos[i]->getValido() && minPuntos >= contadorPuntos[puntos[i]->getEtiqueta()]) {
            puntos[i]->setEtiqueta(0);
        }
    }

    //Contamos cuantas etiquetas nos quedan
    int cont2 = 0;
    for (int i = 0; i < contadorPuntos.size(); i++) {
        if (minPuntos < contadorPuntos[i]) {
            cont2++;
        }
    }

    return cont2;
}

int Segmentacion::reducirPadresYAplicarEtiquetasDepth(vector<Punto3D *> &puntos, vector<int> &padres, vector<int> &etiquetas, vector<int> &contadorPuntos) {

    //Calculamos el padre raiz de cada etiqueta
    for (int i = 1; i < padres.size(); i++) {
        if (padres[i] != 0) { // Si es raiz no hago nada
            int et = i;
            while (padres[et] != 0) et = padres[et];
            padres[i] = et;
        }
    }

    //Sustituimos la etiqueta por la raiz padre de todos los puntos
    for (int i = 0; i < puntos.size(); i++) {

        if (etiquetas[i] != 0) {
            if (padres[etiquetas[i]] != 0) {
                puntos[i]->setEtiqueta(-padres[etiquetas[i]]);
                contadorPuntos[padres[etiquetas[i]]] = contadorPuntos[padres[etiquetas[i]]] + 1;
            } else {
                puntos[i]->setEtiqueta(-etiquetas[i]);
                contadorPuntos[etiquetas[i]] = contadorPuntos[etiquetas[i]] + 1;
            }
        }


    }

    //Marcamos como etiqueta 0 a todos las etiquetas que tengan menos de minPuntos
    for (int i = 1; i < puntos.size(); i++) {
        if (etiquetas[i] != 0) {
            if (padres[etiquetas[i]] != 0 && minPuntosDepth >= contadorPuntos[padres[etiquetas[i]]]) {
                puntos[i]->setEtiqueta(0);
            } else if (padres[etiquetas[i]] == 0 && minPuntosDepth >= contadorPuntos[etiquetas[i]]) {
                puntos[i]->setEtiqueta(0);
            }
        }
    }

    //Contamos cuantas etiquetas nos quedan
    int cont2 = 0;
    for (int i = 0; i < contadorPuntos.size(); i++) {
        if (minPuntosDepth < contadorPuntos[i]) {
            cont2++;
        }
    }

    return cont2;
}

int Segmentacion::asignarPuntosAPlanos(vector<Punto3D *> &puntos, map<int, Plano3D> &planos, vector<int> &contadorPuntos) {

    for (int i = 1; i < contadorPuntos.size(); i++) {
        if (minPuntos < contadorPuntos[i]) {
            planos.insert(pair<int, Plano3D>(i, Plano3D(i)));
        }
    }

    for (int i = 0; i < puntos.size(); i++) {
        if (puntos[i]->getEtiqueta() != 0) {

            map<int, Plano3D>::iterator plano = planos.find(puntos[i]->getEtiqueta());
            if (plano != planos.end()) {
                (*plano).second.addPunto(puntos[i]);
                puntos[i]->setPlano(&((*plano).second));
            }
        }
    }

    return planos.size();

}

int Segmentacion::asignarPuntosAElementos(vector<Punto3D *> &puntos, map<int, Elemento3D> &elementos, vector<int> &contadorPuntos) {

    for (int i = 1; i < contadorPuntos.size(); i++) {
        if (minPuntosDepth < contadorPuntos[i]) {
            elementos.insert(pair<int, Elemento3D>(-i, Elemento3D(-i)));
        }
    }

    for (int i = 0; i < puntos.size(); i++) {
        if (puntos[i]->getEtiqueta() < 0) {

            map<int, Elemento3D>::iterator elemento = elementos.find(puntos[i]->getEtiqueta());
            if (elemento != elementos.end()) {
                (*elemento).second.addPunto(puntos[i]);
                puntos[i]->setElemento(&((*elemento).second));
            } else {
                //cout << "Etiqueta sin elemento " << puntos[i]->getEtiqueta() << " " << contadorPuntos[puntos[i]->getEtiqueta()] << endl;
            }
        }
    }

    map<int, Elemento3D>::iterator it = elementos.begin();
    while (it != elementos.end()) {
        (*it).second.calcularParametrosDelElemento();
        it++;
    }

    return elementos.size();

}

int Segmentacion::borrarPlanosNoValidos(map<int, Plano3D> &planos) {

    vector<int> borrar;

    map<int, Plano3D>::iterator it = planos.begin();

    while (it != planos.end()) {

        (*it).second.calcularParametrosDelPlano();
        bool valido = (*it).second.calcularSiValido(umbralConvexidad, minPuntos, umbralCosenoNormales);

        if (!valido) {
            (*it).second.liberarPuntos();
            it = planos.erase(it);
        } else {
            it++;
        }

    }

    return planos.size();

}

void Segmentacion::extenderPlanos(vector<Punto3D *> &puntos, int w, int h) {

    //Primer recorrido abajo e derecha
    for (int i = 0; i < w - 1; i++) {
        for (int j = 0; j < h - 1; j++) {

            if (puntos[(j * w) + i]->getValido() && puntos[(j * w) + i]->getPlano() != NULL) {

                float threshold = sumaDelta2 + delta2 * (alfa * puntos[j * w + i]->getDepth() * puntos[j * w + i]->getDepth());

                //Derecha
                if (puntos[(j * w) + (i + 1)]->getValido()) {

                    if (puntos[(j * w) + (i + 1)]->getPlano() == NULL) {

                        if (puntos[(j * w) + i]->getPlano()->getDistanciaAPunto(puntos[(j * w) + (i + 1)]) < threshold) {

                            puntos[(j * w) + i]->getPlano()->addPunto(puntos[(j * w) + (i + 1)]);
                            puntos[(j * w) + (i + 1)]->setPlano(puntos[(j * w) + i]->getPlano());

                        }
                    } else if (puntos[(j * w) + (i + 1)]->getPlano() != puntos[(j * w) + i]->getPlano()) {
                        puntos[(j * w) + (i + 1)]->getPlano()->addPlanoColindante(puntos[(j * w) + i]->getPlano());
                        puntos[(j * w) + i]->getPlano()->addPlanoColindante(puntos[(j * w) + (i + 1)]->getPlano());
                    }

                }

                //Abajo
                if (puntos[((j + 1) * w) + i]->getValido()) {

                    if (puntos[((j + 1) * w) + i]->getPlano() == NULL) {

                        if (puntos[(j * w) + i]->getPlano()->getDistanciaAPunto(puntos[((j + 1) * w) + i]) < threshold) {

                            puntos[(j * w) + i]->getPlano()->addPunto(puntos[((j + 1) * w) + i]);
                            puntos[((j + 1) * w) + i]->setPlano(puntos[(j * w) + i]->getPlano());

                        }
                    } else if (puntos[((j + 1) * w) + i]->getPlano() != puntos[(j * w) + i]->getPlano()) {
                        puntos[((j + 1) * w) + i]->getPlano()->addPlanoColindante(puntos[(j * w) + i]->getPlano());
                        puntos[(j * w) + i]->getPlano()->addPlanoColindante(puntos[((j + 1) * w) + i]->getPlano());
                    }
                }
            }
        }
    }

    //Segundo recorrido arriba e izquierda
    for (int i = w - 1; i > 0; i--) {
        for (int j = h - 1; j > 0; j--) {

            if (puntos[(j * w) + i]->getValido() && puntos[(j * w) + i]->getPlano() != NULL) {

                float threshold = sumaDelta2 + delta2 * (alfa * puntos[j * w + i]->getDepth() * puntos[j * w + i]->getDepth());

                //Izquierda
                if (puntos[(j * w) + (i - 1)]->getValido()) {

                    if (puntos[(j * w) + (i - 1)]->getPlano() == NULL) {

                        if (puntos[(j * w) + i]->getPlano()->getDistanciaAPunto(puntos[(j * w) + (i - 1)]) < threshold) {

                            puntos[(j * w) + i]->getPlano()->addPunto(puntos[(j * w) + (i - 1)]);
                            puntos[(j * w) + (i - 1)]->setPlano(puntos[(j * w) + i]->getPlano());

                        }
                    } else if (puntos[(j * w) + (i - 1)]->getPlano() != puntos[(j * w) + i]->getPlano()) {
                        puntos[(j * w) + (i - 1)]->getPlano()->addPlanoColindante(puntos[(j * w) + i]->getPlano());
                        puntos[(j * w) + i]->getPlano()->addPlanoColindante(puntos[(j * w) + (i - 1)]->getPlano());
                    }

                }

                //Arriba
                if (puntos[((j - 1) * w) + i]->getValido()) {

                    if (puntos[((j - 1) * w) + i]->getPlano() == NULL) {

                        if (puntos[(j * w) + i]->getPlano()->getDistanciaAPunto(puntos[((j - 1) * w) + i]) < threshold) {

                            puntos[(j * w) + i]->getPlano()->addPunto(puntos[((j - 1) * w) + i]);
                            puntos[((j - 1) * w) + i]->setPlano(puntos[(j * w) + i]->getPlano());

                        }
                    } else if (puntos[((j - 1) * w) + i]->getPlano() != puntos[(j * w) + i]->getPlano()) {
                        puntos[((j - 1) * w) + i]->getPlano()->addPlanoColindante(puntos[(j * w) + i]->getPlano());
                        puntos[(j * w) + i]->getPlano()->addPlanoColindante(puntos[((j - 1) * w) + i]->getPlano());
                    }
                }
            }
        }
    }

}

void Segmentacion::recalcularParametrosDelPlano(map<int, Plano3D> &planos) {
    map<int, Plano3D>::iterator it = planos.begin();

    while (it != planos.end()) {
        (*it).second.calcularParametrosDelPlano();
        it++;
    }
}

int Segmentacion::combinarPlanos(map<int, Plano3D> &planos) {

    vector<int> borrar;
    set<int> planosEliminados;

    map<int, Plano3D>::iterator it = planos.begin();

    while (it != planos.end()) {

        if (planosEliminados.find((*it).first) == planosEliminados.end()) {

            Plano3D *plano = &((*it).second);
            map<int, Plano3D>::iterator itAux = it;
            map<int, Plano3D>::iterator itCol = ++it;
            it = itAux;

            while (itCol != planos.end()) {

                Plano3D *planoCol = &((*itCol).second);
                bool combinar = false;

                if (plano->isColindante(planoCol) && planosEliminados.find(planoCol->getEtiqueta()) == planosEliminados.end()) {

                    if (plano->getDotProductPlano(planoCol) > umbralCosenoComb) {

                        //float distMedia = (plano->getDistanciaAPuntoCentralDelPlano(planoCol)
                        //	+ planoCol->getDistanciaAPuntoCentralDelPlano(plano))
                        //	/ 2;

                        if (plano->getNDMenosND(planoCol) < umbralDistanciaComb) {
                            combinar = true;
                        }
                    }
                }

                if (combinar) {
                    plano->combinarPlanos(planoCol);
                    planosEliminados.insert(planoCol->getEtiqueta());
                    //Volvemos a buscar vecinos
                    itAux = it;
                    itCol = ++it;
                    it = itAux;

                } else {
                    itCol++;
                }
            }
        }

        it++;

    }

    set<int>::iterator itBorrado = planosEliminados.begin();

    while (itBorrado != planosEliminados.end()) {
        planos.erase(*itBorrado);
        itBorrado++;
    }

    return planos.size();

}

void Segmentacion::detectarSuelo(map<int, Plano3D> &planos, set<int> &suelo) {
    float yMin = numeric_limits<float>::min();

    //Al pasar a Tango hay que comprobar que el suelo es plano en la y

    map<int, Plano3D>::iterator it = planos.begin();

    while (it != planos.end()) {
        if ((*it).second.getCY() < yMin) {
            yMin = (*it).second.getCY();
        }
        it++;
    }

    it = planos.begin();
    while (it != planos.end()) {
        if (abs((*it).second.getCY() - yMin) < 0.1) {//10 centimetros de umbral
            suelo.insert((*it).first);
        }
        it++;
    }

}

void Segmentacion::detectarRelevantes(vector<Punto3D *> puntos, map<int, Plano3D> &planos, map<int, Elemento3D> &elementos, set<int> &suelo, set<int> &relevantes) {

    float maxX = max(getMaxX(puntos), abs(getMinX(puntos)));
    float maxY = max(getMaxY(puntos), abs(getMinY(puntos)));
    float maxZ = getMaxZ(puntos);
    int tamImagen = puntos.size();

    vector<int> etiquetas = vector<int>();
    vector<float> puntuaciones = vector<float>();

    map<int, Plano3D>::iterator itPlano = planos.begin();
    while (itPlano != planos.end()) {
        if (suelo.find((*itPlano).first) == suelo.end()) {
            etiquetas.push_back((*itPlano).first);
            Elemento3D* ele = &(*itPlano).second;
            puntuaciones.push_back(puntuarElemento(ele, tamImagen, maxX, maxY, maxZ));
        }
        itPlano++;
    }

    map<int, Elemento3D>::iterator itElem = elementos.begin();
    while (itElem != elementos.end()) {
        etiquetas.push_back((*itElem).first);
        Elemento3D* ele = &(*itElem).second;
        puntuaciones.push_back(puntuarElemento(ele, tamImagen, maxX, maxY, maxZ));
        itElem++;
    }

    for (int i = 0; i < (etiquetas.size() - 1); i++) {
        for (int j = 0; j < etiquetas.size() - i - 1; j++) {
            if (puntuaciones[j] < puntuaciones[j + 1]) {
                float auxF = puntuaciones[j];
                puntuaciones[j] = puntuaciones[j + 1];
                puntuaciones[j + 1] = auxF;

                int auxI = etiquetas[j];
                etiquetas[j] = etiquetas[j + 1];
                etiquetas[j + 1] = auxI;
            }
        }
    }

    for (int i = 0; i < etiquetas.size() && i < nRelevantes; i++) {
        relevantes.insert(etiquetas[i]);
    }

}


float Segmentacion::puntuarElemento(Elemento3D *elemento, int escalaTamaño, float maxX, float maxY, float maxZ) {

    float puntosTamaño = ((elemento->getNumeroPuntos() * 1.0) / escalaTamaño) * multiplicadorTamaño;
    float puntosPosicion = ((maxX - abs(elemento->getCX()) / maxX) * (maxY - abs(elemento->getCY()) / maxY)) * multiplicadorPosicion;
    float puntosDistancia = (abs(maxZ - abs(elemento->getCZ())) / maxZ) * multiplicadorDistancia;

    return puntosTamaño + puntosPosicion + puntosDistancia;
}

bool Segmentacion::comparadorProfundidad(Punto3D *punto1, Punto3D *punto2) {

    float media = (punto1->getDepth() + punto2->getDepth()) / 2;
    float threshold = sumaDeltaDepth + deltaDepth * (alfa * media * media);

    return abs(punto1->getDepth() - punto2->getDepth()) < threshold;

}


bool Segmentacion::comparadorNormales1(Punto3D *punto1, Punto3D *punto2) {

    //cout << punto1->getNDMenosND(punto2) << endl;

    return punto1->getDotProduct(punto2) > distNormal && punto1->getNDMenosND(punto2) < distRange;

}

void Segmentacion::colorearPorProfundidad(vector<Punto3D *> &puntos, vector<unsigned char> &imagen) {

    float maxD = getMaxDepth(puntos);
    float umbral = maxD / 8;


    for (int i = 0; i < puntos.size(); i++) {

        if (puntos[i]->getValido()) {

            if (puntos[i]->getDepth() < umbral) {
                imagen[i * 3 + 0] = colores[0][0];
                imagen[i * 3 + 1] = colores[0][1];
                imagen[i * 3 + 2] = colores[0][2];
            } else if (puntos[i]->getDepth() < umbral * 2) {
                imagen[i * 3 + 0] = colores[1][0];
                imagen[i * 3 + 1] = colores[1][1];
                imagen[i * 3 + 2] = colores[1][2];
            } else if (puntos[i]->getDepth() < umbral * 3) {
                imagen[i * 3 + 0] = colores[2][0];
                imagen[i * 3 + 1] = colores[2][1];
                imagen[i * 3 + 2] = colores[2][2];
            } else if (puntos[i]->getDepth() < umbral * 4) {
                imagen[i * 3 + 0] = colores[3][0];
                imagen[i * 3 + 1] = colores[3][1];
                imagen[i * 3 + 2] = colores[3][2];
            } else if (puntos[i]->getDepth() < umbral * 5) {
                imagen[i * 3 + 0] = colores[4][0];
                imagen[i * 3 + 1] = colores[4][1];
                imagen[i * 3 + 2] = colores[4][2];
            } else if (puntos[i]->getDepth() < umbral * 6) {
                imagen[i * 3 + 0] = colores[5][0];
                imagen[i * 3 + 1] = colores[5][1];
                imagen[i * 3 + 2] = colores[5][2];
            } else if (puntos[i]->getDepth() < umbral * 7) {
                imagen[i * 3 + 0] = colores[6][0];
                imagen[i * 3 + 1] = colores[6][1];
                imagen[i * 3 + 2] = colores[6][2];
            } else {
                imagen[i * 3 + 0] = colores[7][0];
                imagen[i * 3 + 1] = colores[7][1];
                imagen[i * 3 + 2] = colores[7][2];
            }
        } else {
            imagen[i * 3 + 0] = 0;
            imagen[i * 3 + 1] = 0;
            imagen[i * 3 + 2] = 0;
        }
    }
}

void Segmentacion::colorearPorEtiquetaRelevantes(vector<Punto3D *> &puntos, vector<unsigned char> &imagen, set<int> &suelo, set<int> &relevantes) {

    int contador = 0;
    map<int, vector<int>> paleta;
    vector<int> blanco = {255, 255, 255};

    //Asignamos color blanco al suelo
    set<int>::iterator it = suelo.begin();
    while (it != suelo.end()) {
        int id = *it;
        paleta.insert(pair<int, vector<int>>(id, blanco));
        it++;
    }

    for (int i = 0; i < puntos.size(); i++) {

        if (puntos[i]->getValido() && puntos[i]->getEtiqueta() != 0) {

            if (suelo.find(puntos[i]->getEtiqueta()) != suelo.end() || relevantes.find(puntos[i]->getEtiqueta()) != relevantes.end()) {

                vector<int> color;
                map<int, vector<int>>::iterator it = paleta.find(puntos[i]->getEtiqueta());

                if (it != paleta.end()) {
                    color = it->second;
                } else {
                    color = vector<int>(3);
                    color[0] = (rand() % 100) + 100;
                    color[1] = (rand() % 100) + 100;
                    color[2] = (rand() % 100) + 100;
                    paleta.insert(pair<int, vector<int>>(puntos[i]->getEtiqueta(), color));
                }

                imagen[i * 3 + 0] = color[0];
                imagen[i * 3 + 1] = color[1];
                imagen[i * 3 + 2] = color[2];
            }
        }
    }
}

void Segmentacion::colorearPorEtiqueta(vector<Punto3D *> &puntos, vector<unsigned char> &imagen) {

    int contador = 0;
    map<int, vector<int>> paleta;

    for (int i = 0; i < puntos.size(); i++) {

        if (puntos[i]->getValido() && puntos[i]->getEtiqueta() != 0) {

            vector<int> color;
            map<int, vector<int>>::iterator it = paleta.find(puntos[i]->getEtiqueta());

            if (it != paleta.end()) {
                color = it->second;
            } else {
                color = vector<int>(3);
                color[0] = (rand() % 100) + 100;
                color[1] = (rand() % 100) + 100;
                color[2] = (rand() % 100) + 100;
                paleta.insert(pair<int, vector<int>>(puntos[i]->getEtiqueta(), color));
            }

            imagen[i * 3 + 0] = color[0];
            imagen[i * 3 + 1] = color[1];
            imagen[i * 3 + 2] = color[2];
        }
    }
}


void Segmentacion::colorearPorNormales(vector<Punto3D *> &puntos, vector<unsigned char> &imagen) {

    for (int i = 0; i < puntos.size(); i++) {

        if (puntos[i]->getValido()) {

            imagen[i * 3 + 0] = (puntos[i]->getNX() + 1) * 128;
            imagen[i * 3 + 1] = (puntos[i]->getNY() + 1) * 128;
            imagen[i * 3 + 2] = (puntos[i]->getNZ() + 1) * 128;
        }
    }

}

void Segmentacion::colorearPorNormalesX(vector<Punto3D *> &puntos, vector<unsigned char> &imagen) {

    for (int i = 0; i < puntos.size(); i++) {

        if (puntos[i]->getValido()) {

            imagen[i * 3 + 0] = imagen[i * 3 + 1] = imagen[i * 3 + 2] = (puntos[i]->getNX() + 1) * 128;
        }
    }

}

void Segmentacion::colorearPorNormalesY(vector<Punto3D *> &puntos, vector<unsigned char> &imagen) {

    for (int i = 0; i < puntos.size(); i++) {

        if (puntos[i]->getValido()) {

            imagen[i * 3 + 0] = imagen[i * 3 + 1] = imagen[i * 3 + 2] = (puntos[i]->getNY() + 1) * 128;
        }
    }

}

void Segmentacion::colorearPorNormalesZ(vector<Punto3D *> &puntos, vector<unsigned char> &imagen) {

    for (int i = 0; i < puntos.size(); i++) {

        if (puntos[i]->getValido()) {

            imagen[i * 3 + 0] = imagen[i * 3 + 1] = imagen[i * 3 + 2] = (puntos[i]->getNZ() + 1) * 128;
        }
    }

}

void Segmentacion::colorearPorVecinos(vector<int> &vecinos, vector<unsigned char> &imagen) {

    for (int i = 0; i < vecinos.size(); i++) {
        imagen[i * 3 + 0] = vecinos[i] * 5;
        imagen[i * 3 + 1] = vecinos[i] * 5;
        imagen[i * 3 + 2] = vecinos[i] * 5;
    }
}

void Segmentacion::convolucionProfundidadGaussiana3(vector<Punto3D *> &puntos, int w, int h) {

    const double gaussian3[3][3] = {{0.077847, 0.123317, 0.077847},
                                    {0.123317, 0.195346, 0.123317},
                                    {0.077847, 0.123317, 0.077847}};

    vector<float> depths = vector<float>(w * h, 0);

    int k = 1;

    for (int j = k; j < h - k; j++) {
        for (int i = k; i < w - k; i++) {

            Punto3D *punto = puntos[j * w + i];

            float nuevo = 0;
            float peso = 0;

            for (int m = -k; m <= k; m++) {
                for (int n = -k; n <= k; n++) {
                    if (puntos[((j + n) * w) + (i + m)]->getValido()) {
                        nuevo += puntos[((j + n) * w) + (i + m)]->getDepth() * gaussian3[m + k][n + k];
                        peso += gaussian3[m + k][n + k];
                    }
                }
            }

            depths[j * w + i] = nuevo / peso;

        }
    }

    for (int i = 0; i < puntos.size(); i++) {
        puntos[i]->setDepth(depths[i]);
        if (puntos[i]->getDepth() > 0) {
            puntos[i]->setValido(true);
        }

    }
}

void Segmentacion::convolucionProfundidadGaussiana5(vector<Punto3D *> &puntos, int w, int h) {

    const double gaussian3[5][5] = {{0.003765, 0.015019, 0.023792, 0.015019, 0.003765},
                                    {0.015019, 0.059912, 0.094907, 0.059912, 0.015019},
                                    {0.023792, 0.094907, 0.150342, 0.094907, 0.023792},
                                    {0.015019, 0.059912, 0.094907, 0.059912, 0.015019},
                                    {0.003765, 0.015019, 0.023792, 0.015019, 0.003765}};

    vector<float> depths = vector<float>(w * h, 0);

    int k = 2;

    for (int j = k; j < h - k; j++) {
        for (int i = k; i < w - k; i++) {

            Punto3D *punto = puntos[j * w + i];

            float nuevo = 0;
            float peso = 0;

            for (int m = -k; m <= k; m++) {
                for (int n = -k; n <= k; n++) {
                    if (puntos[((j + n) * w) + (i + m)]->getValido()) {
                        nuevo += puntos[((j + n) * w) + (i + m)]->getDepth() * gaussian3[m + k][n + k];
                        peso += gaussian3[m + k][n + k];
                    }
                }
            }

            depths[j * w + i] = nuevo / peso;

        }
    }

    for (int i = 0; i < puntos.size(); i++) {
        puntos[i]->setDepth(depths[i]);
        if (puntos[i]->getDepth() > 0) {
            puntos[i]->setValido(true);
        }

    }
}


void Segmentacion::imprimirNumero(vector<unsigned char> &imagen, int n, int w, int h) {

    unsigned n0[28] = {0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};
    unsigned n1[28] = {0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1};
    unsigned n2[28] = {1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1};
    unsigned n3[28] = {1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1};
    unsigned n4[28] = {1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1};
    unsigned n5[28] = {1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1};
    unsigned n6[28] = {0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};
    unsigned n7[28] = {1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1};
    unsigned n8[28] = {0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};
    unsigned n9[28] = {0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0};

    int wM = 4;
    int hM = 7;
    int x0 = 1;
    int x1 = 6;
    int x2 = 11;
    int y = 1;
    int tam = 4;

    int num2 = n % 10;
    int num1 = (n / 10) % 10;
    int num0 = (n / 100) % 10;

    switch (num0) {
        case 0:
            dibujarNumero(imagen, w, h, n0, wM, hM, x0, y, tam, 255, 3);
            break;
        case 1:
            dibujarNumero(imagen, w, h, n1, wM, hM, x0, y, tam, 255, 3);
            break;
        case 2:
            dibujarNumero(imagen, w, h, n2, wM, hM, x0, y, tam, 255, 3);
            break;
        case 3:
            dibujarNumero(imagen, w, h, n3, wM, hM, x0, y, tam, 255, 3);
            break;
        case 4:
            dibujarNumero(imagen, w, h, n4, wM, hM, x0, y, tam, 255, 3);
            break;
        case 5:
            dibujarNumero(imagen, w, h, n5, wM, hM, x0, y, tam, 255, 3);
            break;
        case 6:
            dibujarNumero(imagen, w, h, n6, wM, hM, x0, y, tam, 255, 3);
            break;
        case 7:
            dibujarNumero(imagen, w, h, n7, wM, hM, x0, y, tam, 255, 3);
            break;
        case 8:
            dibujarNumero(imagen, w, h, n8, wM, hM, x0, y, tam, 255, 3);
            break;
        case 9:
            dibujarNumero(imagen, w, h, n9, wM, hM, x0, y, tam, 255, 3);
            break;
    }

    switch (num1) {
        case 0:
            dibujarNumero(imagen, w, h, n0, wM, hM, x1, y, tam, 255, 3);
            break;
        case 1:
            dibujarNumero(imagen, w, h, n1, wM, hM, x1, y, tam, 255, 3);
            break;
        case 2:
            dibujarNumero(imagen, w, h, n2, wM, hM, x1, y, tam, 255, 3);
            break;
        case 3:
            dibujarNumero(imagen, w, h, n3, wM, hM, x1, y, tam, 255, 3);
            break;
        case 4:
            dibujarNumero(imagen, w, h, n4, wM, hM, x1, y, tam, 255, 3);
            break;
        case 5:
            dibujarNumero(imagen, w, h, n5, wM, hM, x1, y, tam, 255, 3);
            break;
        case 6:
            dibujarNumero(imagen, w, h, n6, wM, hM, x1, y, tam, 255, 3);
            break;
        case 7:
            dibujarNumero(imagen, w, h, n7, wM, hM, x1, y, tam, 255, 3);
            break;
        case 8:
            dibujarNumero(imagen, w, h, n8, wM, hM, x1, y, tam, 255, 3);
            break;
        case 9:
            dibujarNumero(imagen, w, h, n9, wM, hM, x1, y, tam, 255, 3);
            break;
    }

    switch (num2) {
        case 0:
            dibujarNumero(imagen, w, h, n0, wM, hM, x2, y, tam, 255, 3);
            break;
        case 1:
            dibujarNumero(imagen, w, h, n1, wM, hM, x2, y, tam, 255, 3);
            break;
        case 2:
            dibujarNumero(imagen, w, h, n2, wM, hM, x2, y, tam, 255, 3);
            break;
        case 3:
            dibujarNumero(imagen, w, h, n3, wM, hM, x2, y, tam, 255, 3);
            break;
        case 4:
            dibujarNumero(imagen, w, h, n4, wM, hM, x2, y, tam, 255, 3);
            break;
        case 5:
            dibujarNumero(imagen, w, h, n5, wM, hM, x2, y, tam, 255, 3);
            break;
        case 6:
            dibujarNumero(imagen, w, h, n6, wM, hM, x2, y, tam, 255, 3);
            break;
        case 7:
            dibujarNumero(imagen, w, h, n7, wM, hM, x2, y, tam, 255, 3);
            break;
        case 8:
            dibujarNumero(imagen, w, h, n8, wM, hM, x2, y, tam, 255, 3);
            break;
        case 9:
            dibujarNumero(imagen, w, h, n9, wM, hM, x2, y, tam, 255, 3);
            break;
    }

}

void Segmentacion::dibujarNumero(vector<unsigned char> &imagen, int w, int h, unsigned num[], int wM, int hM, int x, int y, int tam, int color, int canales) {

    for (int j = 0; j < hM; j++) {
        for (int i = 0; i < wM; i++) {

            if (num[j * wM + i] == 1) {

                for (int n = 0; n < tam; n++) {
                    for (int m = 0; m < tam; m++) {

                        for (int canal = 0; canal < canales; canal++) {

                            imagen[((((((j + y) * tam) + n) * w) + ((i + x) * tam + m)) * canales) + canal] = color;
                        }
                    }
                }
            }
        }
    }
}

void Segmentacion::liberarVector(vector<Punto3D *> vect) {

    for (vector<Punto3D *>::iterator it = vect.begin(); it != vect.end(); ++it) {
        delete (*it);
    }

    vect.clear();

}

float Segmentacion::getMaxDepth(vector<Punto3D *> puntos) {

    float maxD = 0;
    for (vector<Punto3D *>::iterator it = puntos.begin(); it != puntos.end(); ++it) {
        if ((*it)->getDepth() > maxD) {
            maxD = (*it)->getDepth();
        }
    }

    return maxD;
}

float Segmentacion::getMaxZ(vector<Punto3D *> puntos) {

    float maxD = 0;
    for (vector<Punto3D *>::iterator it = puntos.begin(); it != puntos.end(); ++it) {
        if ((*it)->getZ() > maxD) {
            maxD = (*it)->getZ();
        }
    }

    return maxD;
}

float Segmentacion::getMaxY(vector<Punto3D *> puntos) {

    float maxD = 0;
    for (vector<Punto3D *>::iterator it = puntos.begin(); it != puntos.end(); ++it) {
        if ((*it)->getY() > maxD) {
            maxD = (*it)->getY();
        }
    }

    return maxD;
}

float Segmentacion::getMaxX(vector<Punto3D *> puntos) {

    float maxD = 0;
    for (vector<Punto3D *>::iterator it = puntos.begin(); it != puntos.end(); ++it) {
        if ((*it)->getX() > maxD) {
            maxD = (*it)->getX();
        }
    }

    return maxD;
}

float Segmentacion::getMinZ(vector<Punto3D *> puntos) {

    float maxD = numeric_limits<float>::min();
    for (vector<Punto3D *>::iterator it = puntos.begin(); it != puntos.end(); ++it) {
        if ((*it)->getZ() < maxD) {
            maxD = (*it)->getZ();
        }
    }

    return maxD;
}

float Segmentacion::getMinX(vector<Punto3D *> puntos) {

    float maxD = numeric_limits<float>::min();
    for (vector<Punto3D *>::iterator it = puntos.begin(); it != puntos.end(); ++it) {
        if ((*it)->getX() < maxD) {
            maxD = (*it)->getX();
        }
    }

    return maxD;
}

float Segmentacion::getMinY(vector<Punto3D *> puntos) {

    float maxD = numeric_limits<float>::min();
    for (vector<Punto3D *>::iterator it = puntos.begin(); it != puntos.end(); ++it) {
        if ((*it)->getY() < maxD) {
            maxD = (*it)->getY();
        }
    }

    return maxD;
}