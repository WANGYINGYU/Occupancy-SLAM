//
// Created by Yingyu Wang on 18/4/2023.
#include "MyStruct.h"
#include "SubFuncs.h"



std::tuple<Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>> FuncGetI(const Eigen::MatrixXd& Odom, const Eigen::VectorXd& ErrorS)
{
    int nO = Odom.rows() - 1;
    Eigen::ArrayXd Sigma_O(3);
    Sigma_O << 1/(0.05*0.05), 1/(0.05*0.05), 1/(0.02*0.02);
    Eigen::ArrayXd diagSigmaO = Sigma_O.replicate(nO,1);
    Eigen::SparseMatrix<double> IO(3*nO,3*nO);
    IO.setIdentity();
    IO.diagonal() = diagSigmaO;
    double Sigma_S = 1.0;
    int nS = ErrorS.size();
    Eigen::ArrayXd diagSigmaS = Eigen::ArrayXd::Constant(nS,Sigma_S);
    Eigen::SparseMatrix<double> IS(nS,nS);
    IS.setIdentity();
    IS.diagonal() = diagSigmaS;
    return std::make_tuple(IS, IO);
}


double FuncWrap(double alpha) {
    double nu = alpha;
    while (nu > M_PI) {
        nu = nu - 2 * M_PI;
    }
    while (nu < -M_PI) {
        nu = nu + 2 * M_PI;
    }
    return nu;
}

void loadFileData(const std::vector<std::string>& filenames, std::vector<std::vector<double>>& fileData, const ParamStruct& ValParam) {
    // Load the data from each file and store it in the vectors
    int numFiles = filenames.size();
    for (int i = 0; i < numFiles; i++) {
        std::ifstream file(filenames[i]);
        if (file.is_open()) {
            std::vector<double> data;
            float value;
            while (file >> value) {
                data.push_back(value);
            }
            fileData.push_back(data);
            file.close();
        } else {
            std::cout << "Unable to open file: " << filenames[i] << std::endl;
        }
    }
}

void loadTxTData(const std::string& filenames, std::vector<double>& fileData, const ParamStruct& ValParam) {
    // Load the data from each file and store it in the vectors
    std::ifstream file(filenames);
    if (file.is_open()) {
        float value;
        while (file >> value) {
            fileData.push_back(value);
        }
    } else {
        std::cout << "Unable to open file: " << filenames << std::endl;
    }
}

Eigen::VectorXd VectorToEigenVec(const std::vector<double>& data)
{
    Eigen::Map<const Eigen::VectorXd> eigenData(data.data(), data.size());
    return eigenData;
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> FuncInitialiseGridMap(const Eigen::MatrixXd& Pose, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, const ParamStruct& ValParam){
    Eigen::MatrixXd Map = Eigen::MatrixXd::Zero(ValParam.Sizei, ValParam.Sizej);
    Eigen::MatrixXd N = Eigen::MatrixXd::Zero(ValParam.Sizei, ValParam.Sizej);
    int NumPose = Pose.size()/3;
    #pragma omp parallel for
    for (int i = 0; i < NumPose; ++i) {
        Eigen::Rotation2Dd rotation(Pose(i,2));
        Eigen::Matrix2d Ri = rotation.toRotationMatrix();
        Eigen::ArrayXd XYi = ScanXY[i];
        Eigen::ArrayXd Oddi = ScanOdd[i];
        Eigen::Map<Eigen::MatrixXd> MatrixXY(XYi.data(), 2, XYi.size()/2);
        Eigen::Vector2d VectorTrans = Pose.block<>(i,0,1,2).transpose();
        Eigen::MatrixXd Si = (Ri * MatrixXY).colwise() + VectorTrans;
        Eigen::Vector2d Origin(ValParam.OriginX,ValParam.OriginY);
        Eigen::MatrixXd XY3 = ((Si.colwise()-Origin).array() / ValParam.Scale).floor().matrix();
        Eigen::MatrixXd TemGlobal = Eigen::MatrixXd::Zero(ValParam.Sizei, ValParam.Sizej);
        Eigen::MatrixXd TemMapN = Eigen::MatrixXd::Zero(ValParam.Sizei, ValParam.Sizej);
        for (int j = 0; j < Oddi.size(); ++j) {
            int tem_col = XY3(0,j);
            int tem_row = XY3(1,j);
            double tem_val = Oddi[j];
            TemGlobal(tem_row, tem_col) = TemGlobal(tem_row, tem_col) + tem_val;
            TemMapN(tem_row, tem_col) = TemMapN(tem_row, tem_col) + 1;
        }
        Map = Map + TemGlobal;
        N = N + TemMapN;
    }
    return std::make_tuple(Map, N);
}

Eigen::SparseMatrix<double> FuncMapConst(const ParamStruct& ValParam) {
    int Sizei = ValParam.Sizei;
    int Sizej = ValParam.Sizej;
    int NumElement = ((Sizei-1)*(Sizej-1)*2 + Sizei- 1 + Sizej - 1)*2;
    Eigen::VectorXi ID1(NumElement);
    Eigen::VectorXi ID2(NumElement);
    Eigen::VectorXi Val(NumElement);
    int nCnt = 0;

//    #pragma omp parallel for collapse(2)
    for (int i = 0; i < Sizei; i++)
    {
        for (int j = 0; j < Sizej; j++)
        {
            int ij0 = Sizej * i + j;
            int ij1 = Sizej * i + j + 1;
            int ij2 = Sizej * (i + 1) + j;
            if (j + 1 < Sizej) {
                nCnt++;
                ID1(nCnt*2-2) = nCnt-1;
                ID1(nCnt*2-1) = nCnt-1;
                ID2(nCnt*2-2) = ij0;
                ID2(nCnt*2-1) = ij1;
                Val(nCnt*2-2) = 1;
                Val(nCnt*2-1) = -1;
            }

            if (i + 1 < Sizei) {
                nCnt++;
                ID1(nCnt*2-2) = nCnt-1;
                ID1(nCnt*2-1) = nCnt-1;
                ID2(nCnt*2-2) = ij0;
                ID2(nCnt*2-1) = ij2;
                Val(nCnt*2-2) = 1;
                Val(nCnt*2-1) = -1;
            }
        }
    }

    int MaxID1 = ID1.maxCoeff();
    int MaxID2 = ID2.maxCoeff();

    Eigen::SparseMatrix<double> J(MaxID1+1,MaxID2+1);

    igl::sparse(ID1,ID2,Val,MaxID1+1,MaxID2+1,J);
    Eigen::SparseMatrix<double> HH = J.transpose() * J;
    return HH;
}


void FuncSmoothN2(Eigen::MatrixXd& N, const Eigen::SparseMatrix<double>& HH, const ParamStruct& ValParam){
    Eigen::SparseMatrix<double> SparseN = N.sparseView();
    Eigen::VectorXi I, J;
    Eigen::VectorXd Val;
    igl::find(SparseN, I, J ,Val);
    int ni = I.size();
    Eigen::VectorXi ID1(ni);
    ID1.setLinSpaced(ni, 0, ni - 1);
    Eigen::VectorXi ID2(ni);
    ID2 = ValParam.Sizej * I  + J;
    Eigen::VectorXd VecOnes = Eigen::VectorXd::Ones(ni);
    Eigen::SparseMatrix<double> A1;
    igl::sparse(ID1,ID2,VecOnes,ni,ValParam.Sizei * ValParam.Sizej, A1);
    Eigen::SparseMatrix<double> WeightHH = HH * ValParam.WeightSmoothN;
    Eigen::SparseMatrix<double> II = A1.transpose() * A1 + WeightHH;
    Eigen::SparseMatrix<double> EE = (A1.transpose() * Val).sparseView();
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver;
    solver.compute(II);
    Eigen::VectorXd DeltaN = solver.solve(EE);
    N = (Eigen::Map<const Eigen::MatrixXd>(DeltaN.data(), ValParam.Sizej, ValParam.Sizei)).transpose();

}

Eigen::ArrayXd FuncBilinearInterpolation(const Eigen::MatrixXd& Map, const Eigen::MatrixXd& ObjectXY){
    // The input should be the [x,y], i.e. the order is col and row
    int nn = ObjectXY.rows(); // The number of points
    Eigen::ArrayXd res(nn); // The interpolation result
    #pragma omp parallel for
    for (int i = 0; i < nn; i++)
    {
        double x = ObjectXY(i, 0);
        double y = ObjectXY(i, 1);
        int i1 = floor(y);
        int i2 = i1 + 1;
        int j1 = floor(x);
        int j2 = j1 + 1;
        double x1 = j1;
        double x2 = j2;
        double y1 = i1;
        double y2 = i2;
        // Check the boundaries
        if (i1 < 0 || i2 >= Map.rows() || j1 < 0 || j2 >= Map.cols())
        {
            res(i) = 0;
            continue;
        }
        double q11 = Map(i1, j1);
        double q21 = Map(i1, j2);
        double q12 = Map(i2, j1);
        double q22 = Map(i2, j2);
        long double w1 = (x2 - x) * q11 + (x - x1)  * q21;
        long double w2 = (x2 - x) * q12 + (x - x1)  * q22;
        double w = (y2 - y) * w1 + (y - y1) * w2;
        res(i) = w;
    }
    return res;
}

template<typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> FuncGradient(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& M, const T& h)
{
    int rows = M.rows();
    int cols = M.cols();
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Gx(rows, cols);
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Gy(rows, cols);

    // Compute Gx and Gy
    #pragma omp parallel for
    for (int j = 0; j < cols; j++)
    {
        for (int i = 0; i < rows; i++)
        {
            if (i == 0)
            {
                Gx(i, j) = (M(i+1, j) - M(i, j)) / h;
            }
            else if (i == rows - 1)
            {
                Gx(i, j) = (M(i, j) - M(i-1, j)) / h;
            }
            else
            {
                Gx(i, j) = (M(i+1, j) - M(i-1, j)) / (2 * h);
            }
        }
    }

    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            if (j == 0)
            {
                Gy(i, j) = (M(i, j+1) - M(i, j)) / h;
            }
            else if (j == cols - 1)
            {
                Gy(i, j) = (M(i, j) - M(i, j-1)) / h;
            }
            else
            {
                Gy(i, j) = (M(i, j+1) - M(i, j-1)) / (2 * h);
            }
        }
    }

    // Combine Gx and Gy into a single matrix
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> G(rows, cols * 2);
    G << Gx, Gy;
    return G;
}

Eigen::Matrix2d FuncdR2D(const double& phi)
{
    Eigen::Matrix2d dR;
    dR << -sin(phi), cos(phi),
            -cos(phi), -sin(phi);
    return dR;
}

Eigen::Matrix2d FuncTheta2R(const double& phi)
{
    Eigen::Matrix2d R;
    R << cos(phi), -sin(phi),
            sin(phi), cos(phi);
    return R;
}


std::tuple<Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::ArrayXd,Eigen::ArrayXd,double,double> FuncDiffJacobian(const Eigen::MatrixXd& Map, const Eigen::MatrixXd& N, const Eigen::MatrixXd& Pose, const Eigen::MatrixXd& Odom, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, const ParamStruct& ValParam){
    int NumPose = Pose.size()/3;
    // Calculate the gradient of map
    double h = 1.0;
    Eigen::MatrixXd G = FuncGradient(Map,h);
    Eigen::MatrixXd Gv = G.block(0, 0, Map.rows(), Map.cols());
    Eigen::MatrixXd Gu = G.block(0, Map.cols(), Map.rows(), Map.cols());

    int nPts = 0;

    Eigen::ArrayXi JPID1;
    Eigen::ArrayXi JPID2;
    Eigen::ArrayXd JPVal;

    Eigen::ArrayXi JDID1;
    Eigen::ArrayXi JDID2;
    Eigen::ArrayXd JDVal;

    Eigen::ArrayXi JOID1;
    Eigen::ArrayXi JOID2;
    Eigen::ArrayXd JOVal;

    Eigen::ArrayXd ErrorS;
    Eigen::ArrayXd ErrorO;

    for (int i = 0; i < NumPose; ++i) {
        Eigen::Rotation2Dd rotation(Pose(i,2));
        Eigen::Matrix2d Ri = rotation.toRotationMatrix();
        Eigen::ArrayXd XYi = ScanXY[i];
        Eigen::ArrayXd Oddi = ScanOdd[i];
        Eigen::Map<Eigen::MatrixXd> MatrixXY(XYi.data(), 2, XYi.size()/2);


        Eigen::Vector2d VectorTrans = Pose.block<>(i,0,1,2).transpose();
        Eigen::MatrixXd XY2 = (Ri * MatrixXY).colwise() + VectorTrans;
        Eigen::Vector2d Origin(ValParam.OriginX,ValParam.OriginY);
        Eigen::MatrixXd XY3 = ((XY2.colwise()-Origin).array() / ValParam.Scale).matrix();

        // Interpolation
        Eigen::ArrayXd MapInterpXY3 = FuncBilinearInterpolation(Map, XY3.transpose());
        Eigen::ArrayXd NInterpXY3 = FuncBilinearInterpolation(N, XY3.transpose());

        // Calculation of Error w.r.t. Observations Term
        Eigen::ArrayXd Ei = MapInterpXY3.cwiseQuotient(NInterpXY3) - Oddi;

        ErrorS.conservativeResize(ErrorS.size()+Ei.size());
        ErrorS.tail(Ei.size()) = Ei;


        Eigen::ArrayXd GuInterpXY3 = FuncBilinearInterpolation(Gu, XY3.transpose());
        Eigen::ArrayXd GvInterpXY3 = FuncBilinearInterpolation(Gv, XY3.transpose());

        // Calculation of Jacobian of Observation Terms w.r.t. Poses
        Eigen::MatrixXd dMdXY3(2,NInterpXY3.size());
        dMdXY3.row(0) = GuInterpXY3 / NInterpXY3;
        dMdXY3.row(1) = GvInterpXY3 / NInterpXY3;

        Eigen::Matrix2d dR = FuncdR2D(Pose(i,2));
        Eigen::MatrixXd dXY3dR = dR.transpose() * MatrixXY / ValParam.Scale;
        Eigen::MatrixXd dMdR = (dMdXY3.array() * dXY3dR.array()).matrix().colwise().sum();
        Eigen::MatrixXd dMdT = dMdXY3 / ValParam.Scale;

        Eigen::MatrixXd dMdP(3,dMdR.size());
        dMdP << dMdT, dMdR;

        int nPtsi = Oddi.size();
        Eigen::ArrayXi IDi = Eigen::ArrayXi::LinSpaced(nPtsi, nPts, nPts + nPtsi -1);
        nPts = nPts+nPtsi;
        Eigen::MatrixXi dEdPID1 = IDi.replicate<1,3>();

        Eigen::ArrayXi IDCol = Eigen::ArrayXi::LinSpaced(3, 3*i, 3*(i+1)-1);
        Eigen::MatrixXi dEdPID2 = IDCol.replicate(1, nPtsi).transpose();

        Eigen::ArrayXi JPID1i = Eigen::Map<Eigen::ArrayXi>(dEdPID1.data(), dEdPID1.size());
        Eigen::ArrayXi JPID2i = Eigen::Map<Eigen::ArrayXi>(dEdPID2.data(), dEdPID2.size());
        Eigen::MatrixXd dMdPTranpose = dMdP.transpose();
        Eigen::ArrayXd JPVali = Eigen::Map<Eigen::ArrayXd>(dMdPTranpose.data(), dMdPTranpose.size());

        JPID1.conservativeResize(JPID1.size()+JPID1i.size());
        JPID1.tail(JPID1i.size()) = JPID1i;
        JPID2.conservativeResize(JPID2.size()+JPID2i.size());
        JPID2.tail(JPID2i.size()) = JPID2i;
        JPVal.conservativeResize(JPVal.size()+JPVali.size());
        JPVal.tail(JPVali.size()) = JPVali;


        // Calculation of Jacobian of Observation Terms w.r.t. Map
        Eigen::MatrixXi XY3Floor = XY3.array().floor().cast<int>().matrix();

        Eigen::ArrayXd u = XY3.row(0);
        Eigen::ArrayXd v = XY3.row(1);

        Eigen::ArrayXd u1 = XY3Floor.row(0).cast<double>();
        Eigen::ArrayXd v1 = XY3Floor.row(1).cast<double>();

        Eigen::ArrayXd a = (v1+1-v) * (u1+1-u) / NInterpXY3;
        Eigen::ArrayXd b = (v-v1) * (u1+1-u) / NInterpXY3;
        Eigen::ArrayXd c = (v1+1-v) * (u-u1) / NInterpXY3;
        Eigen::ArrayXd d = (v-v1) * (u-u1) / NInterpXY3;

        Eigen::MatrixXd dEdM(4,u.size());
        dEdM << Eigen::Map<Eigen::ArrayXXd>(a.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXd>(b.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXd>(c.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXd>(d.data(), 1, u.size());

        Eigen::MatrixXd dEdMID2(4,u.size());

        a = ValParam.Sizej * v1 + u1;
        b = ValParam.Sizej * (v1 + 1) + u1;
        c = ValParam.Sizej * v1 + u1 + 1;
        d = ValParam.Sizej * (v1 + 1) + u1 +1;

        dEdMID2 << Eigen::Map<Eigen::ArrayXXd>(a.data(), 1, u.size()),
                   Eigen::Map<Eigen::ArrayXXd>(b.data(), 1, u.size()),
                   Eigen::Map<Eigen::ArrayXXd>(c.data(), 1, u.size()),
                   Eigen::Map<Eigen::ArrayXXd>(d.data(), 1, u.size());

        Eigen::MatrixXi dEdMID1 = IDi.replicate<1,4>();
        Eigen::ArrayXi JDID1i = Eigen::Map<Eigen::ArrayXi>(dEdMID1.data(), dEdMID1.size());

        Eigen::MatrixXi dEdMID2Tranp = dEdMID2.transpose().cast<int>();
        Eigen::ArrayXi JDID2i = Eigen::Map<Eigen::ArrayXi>(dEdMID2Tranp.data(), dEdMID2Tranp.size());

        Eigen::MatrixXd dEdMTranp = dEdM.transpose();
        Eigen::ArrayXd JDVali = Eigen::Map<Eigen::ArrayXd>(dEdMTranp.data(), dEdMTranp.size());

        JDID1.conservativeResize(JDID1.size()+JDID1i.size());
        JDID1.tail(JDID1i.size()) = JDID1i;
        JDID2.conservativeResize(JDID2.size()+JDID2i.size());
        JDID2.tail(JDID2i.size()) = JDID2i;
        JDVal.conservativeResize(JDVal.size()+JDVali.size());
        JDVal.tail(JDVali.size()) = JDVali;
        // Calculate Jacobian w.r.t. the odometry term
        if (i < NumPose-1){
            int Cnti = 3 * i;
            Eigen::ArrayXd P1 =  Pose.row(i);
            Eigen::ArrayXd P2 =  Pose.row(i+1);
            Eigen::MatrixXd dT = Ri.transpose() * (P2.head(2) - P1.head(2)).matrix();
            double dPhi = P2(2)-P1(2);
            while (dPhi> M_PI || dPhi< - M_PI){
                dPhi = FuncWrap(dPhi);
            }
            Eigen::MatrixXd EOi(3,1);
            EOi.block(0,0,2,1) = dT;
            EOi(2,0) = dPhi;
            EOi = EOi - Odom.row(i+1).transpose();
            while (EOi(2,0)> M_PI || EOi(2,0)< - M_PI){
                EOi(2,0) = FuncWrap(EOi(2,0));
            }
            ErrorO.conservativeResize(ErrorO.size()+EOi.size());
            ErrorO.tail(EOi.size()) = EOi;

            Eigen::Matrix2d dTdT1 = -Ri.transpose();
            Eigen::Matrix2d dTdT2 = Ri.transpose();
            Eigen::MatrixXd dTdPhi1 = dR * (P2.head(2) - P1.head(2)).matrix();
            double dPhid1 = -1.0;
            double dPhid2 = 1.0;
            Eigen::ArrayXi aa = Eigen::ArrayXi::LinSpaced(3, 3*i, 3*i+2);
            Eigen::ArrayXi bb = Eigen::ArrayXi::LinSpaced(3, 3*(i+1), 3*(i+1)+2);
            Eigen::ArrayXi JOID1i(12);
            JOID1i << Cnti,Cnti,Cnti+1,Cnti+1,Cnti,Cnti,Cnti+1,Cnti+1,Cnti,Cnti+1,Cnti+2,Cnti+2;
            Eigen::ArrayXi JOID2i(12);
            JOID2i << aa.head(2), aa.head(2), bb.head(2), bb.head(2), aa(2), aa(2), aa(2), bb(2);


            Eigen::ArrayXd JOVali(12);
            JOVali.head(2) = dTdT1.row(0);
            JOVali.segment(2,2) = dTdT1.row(1);
            JOVali.segment(4,2) = dTdT2.row(0);
            JOVali.segment(6,2) = dTdT2.row(1);
            JOVali(8) = dTdPhi1(0,0);
            JOVali(9) = dTdPhi1(1,0);
            JOVali(10) = dPhid1;
            JOVali(11) = dPhid2;

            JOID1.conservativeResize(JOID1.size()+JOID1i.size());
            JOID1.tail(JOID1i.size()) = JOID1i;
            JOID2.conservativeResize(JOID2.size()+JOID2i.size());
            JOID2.tail(JOID2i.size()) = JOID2i;
            JOVal.conservativeResize(JOVal.size()+JOVali.size());
            JOVal.tail(JOVali.size()) = JOVali;
        }
    }

    Eigen::SparseMatrix<double> JP(ErrorS.size(),3*NumPose), JD(ErrorS.size(),ValParam.Sizei*ValParam.Sizej), JO(3*(NumPose-1),3*NumPose);
    std::thread t1([&]() {
        igl::sparse(JPID1, JPID2, JPVal, ErrorS.size(), 3 * NumPose, JP);
    });
    std::thread t2([&]() {
        igl::sparse(JDID1, JDID2, JDVal, ErrorS.size(), ValParam.Sizei * ValParam.Sizej, JD);
    });
    std::thread t3([&]() {
        igl::sparse(JOID1, JOID2, JOVal, 3 * (NumPose - 1), 3 * NumPose, JO);
    });
    t1.join();
    t2.join();
    t3.join();

    JP.makeCompressed();
    JD.makeCompressed();
    JO.makeCompressed();

    auto [IS,IO] = FuncGetI(Odom,ErrorS);
    IO.makeCompressed();
    IS.makeCompressed();

    Eigen::MatrixXd MatErrorS = Eigen::MatrixXd::Map(ErrorS.data(), ErrorS.size(), 1);
    Eigen::MatrixXd MatErrorO = Eigen::MatrixXd::Map(ErrorO.data(), ErrorO.size(), 1);

    double SumErrorS = (MatErrorS.transpose() * IS * MatErrorS)(0,0);
    double SumErrorO = (ValParam.WeightO * MatErrorO.transpose() * IO * MatErrorO)(0,0);
    double SumError = SumErrorS + SumErrorO;
    double MeanError = SumErrorS/ErrorS.size() + SumErrorO/ErrorO.size();

    return std::make_tuple(JP,JD,JO,IS,IO,ErrorS,ErrorO,SumError,MeanError);
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd, double, double, double, double> FuncDelta(const Eigen::MatrixXd& Map, const Eigen::MatrixXd& Odom, const Eigen::SparseMatrix<double>& JP, const Eigen::SparseMatrix<double>& JD, const Eigen::SparseMatrix<double>& JO, const Eigen::ArrayXd& ErrorS, const Eigen::ArrayXd& ErrorO, const Eigen::SparseMatrix<double>& IS, const Eigen::SparseMatrix<double>& IO, const Eigen::SparseMatrix<double>& WeightHH, const ParamStruct& ValParam)
{
    Eigen::SparseMatrix<double> JPSlice = JP.block(0, 3, JP.rows(), JP.cols()-3);
    Eigen::SparseMatrix<double> JOSlice = JO.block(0, 3, JO.rows(), JO.cols()-3);
    JPSlice.makeCompressed();
    JOSlice.makeCompressed();

    Eigen::SparseMatrix<double> U = JPSlice.transpose() * JPSlice + ValParam.WeightO * JOSlice.transpose() * IO * JOSlice;
    Eigen::SparseMatrix<double> V = JD.transpose() * JD + WeightHH;
    Eigen::SparseMatrix<double> W = JPSlice.transpose() * JD;

    Eigen::VectorXd VecErrorS = Eigen::VectorXd::Map(ErrorS.data(), ErrorS.size());
    Eigen::VectorXd VecErrorO = Eigen::VectorXd::Map(ErrorO.data(), ErrorO.size());
    Eigen::VectorXd EP = -JPSlice.transpose() * IS * VecErrorS - ValParam.WeightO * JOSlice.transpose() * IO * VecErrorO;
    Eigen::VectorXd ED = -JD.transpose() * IS * VecErrorS;
    Eigen::VectorXd XH0 = Map.transpose().reshaped(Map.size(),1);
    Eigen::VectorXd EH = -WeightHH * XH0;
    Eigen::VectorXd EDEH = ED + EH;

    Eigen::SparseMatrix<double> UW = igl::cat(2, U, W);
    Eigen::SparseMatrix<double> WT = W.transpose();
    Eigen::SparseMatrix<double> WV = igl::cat(2, WT, V);
    Eigen::SparseMatrix<double> II = igl::cat(1, UW, WV);
    Eigen::VectorXd EE = igl::cat(1,EP,EDEH);

    II.makeCompressed();
    Eigen::initParallel();
//    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    Eigen::ConjugateGradient <Eigen::SparseMatrix<double>, Eigen::Upper|Eigen::Lower> solver;
    int MaxNum = floor(sqrt(II.rows()));
    if (MaxNum>200){MaxNum=200;}
    solver.setMaxIterations(MaxNum);
    solver.setTolerance(0.005);
    solver.compute(II);
    Eigen::VectorXd Delta = solver.solve(EE);

    Eigen::VectorXd DeltaP = Delta.head(JPSlice.cols());
    Eigen::VectorXd DeltaD = Delta.tail(Delta.size() - JPSlice.cols());

    double SumDelta = Delta.transpose() * Delta;
    double MeanDelta = SumDelta / Delta.size();
    double SumDeltaP = DeltaP.transpose() * DeltaP;
    double MeanDeltaP = SumDeltaP / DeltaP.size();

    return std::make_tuple(DeltaP,DeltaD,SumDelta,MeanDelta,SumDeltaP,MeanDeltaP);
}

void FuncUpdate(Eigen::MatrixXd& Map, Eigen::MatrixXd& Pose, const Eigen::VectorXd& DeltaP, const Eigen::VectorXd& DeltaD){
    Eigen::MatrixXd DeltaP2 = Eigen::Map<const Eigen::MatrixXd>(DeltaP.data(), 3, DeltaP.size()/3).transpose();
    Pose.block(1, 0, Pose.rows()-1, Pose.cols()) += DeltaP2;
    int Size_i = Map.rows();
    int Size_j = Map.cols();
    Eigen::MatrixXd DeltaD2 = Eigen::Map<const Eigen::MatrixXd>(DeltaD.data(), Size_j, Size_i).transpose();
    Map += DeltaD2;
}

void FuncUpdateMapN(Eigen::MatrixXd& N, const Eigen::MatrixXd& Pose, const std::vector<Eigen::ArrayXd>& ScanXY, const ParamStruct& ValParam){
    N = Eigen::MatrixXd::Zero(ValParam.Sizei, ValParam.Sizej);
    int NumPose = Pose.size()/3;
    #pragma omp parallel for
    for (int i = 0; i < NumPose; ++i) {
        Eigen::Rotation2Dd rotation(Pose(i,2));
        Eigen::Matrix2d Ri = rotation.toRotationMatrix();
        Eigen::ArrayXd XYi = ScanXY[i];
        Eigen::Map<Eigen::MatrixXd> MatrixXY(XYi.data(), 2, XYi.size()/2);
        Eigen::Vector2d VectorTrans = Pose.block<>(i,0,1,2).transpose();
        Eigen::MatrixXd Si = (Ri * MatrixXY).colwise() + VectorTrans;
        Eigen::Vector2d Origin(ValParam.OriginX,ValParam.OriginY);
        Eigen::MatrixXd XY3 = ((Si.colwise()-Origin).array() / ValParam.Scale).round().matrix();

        Eigen::MatrixXd TemMapN = Eigen::MatrixXd::Zero(ValParam.Sizei, ValParam.Sizej);
        for (int j = 0; j < XY3.cols(); ++j) {
            int tem_col = XY3(0,j);
            int tem_row = XY3(1,j);
            TemMapN(tem_row, tem_col) = TemMapN(tem_row, tem_col) + 1;
        }
        N = N + TemMapN;
    }
}

void FuncLowSampleScan(const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, std::vector<Eigen::ArrayXd>& LowScanXY, std::vector<Eigen::ArrayXd>& LowScanOdd, const ParamStruct& ValParam){
    if ((ValParam.ModeMulti == false) || (ValParam.DownRate ==1)){
        LowScanXY = ScanXY;
        LowScanOdd = ScanOdd;
    }
    else{
        for (int i = 0; i < ScanXY.size(); ++i) {
            Eigen::ArrayXd Oddi = ScanOdd[i];
            Eigen::ArrayXd XYi = ScanXY[i];
            Eigen::VectorXi IdHit;
            igl::find(Oddi > 0, IdHit);
            int mm = IdHit.size();
            std::vector<Eigen::VectorXi> VecLabel(mm-1);
            for (int j = 0; j < mm-1; ++j) {
                double d = IdHit(j+1) - IdHit(j);
                double Lowd = d / ValParam.DownRate;
                Lowd = std::max(Lowd, 1.0);
                Eigen::VectorXi Val;
                if(Lowd == 1.0)
                {
                    Val.resize(2);
                    Val(0) = IdHit(j);
                    Val(1) = IdHit(j+1) - 1;
                }
                else
                {
                    int nn = std::floor((IdHit(j+1)-1-IdHit(j))/ValParam.DownRate) + 1;
                    Val.resize(nn);
                    for(int k = 0; k < nn; k++)
                    {
                        Val(k) = IdHit(j) + k * ValParam.DownRate;
                    }
                }
                VecLabel[j] = Val;
            }
            int TotalSize = 0;
            for (const auto& vec : VecLabel) {
                TotalSize += vec.size();
            }
            Eigen::VectorXi ConcatLabel(TotalSize);
            int offset = 0;
            for (const auto& vec : VecLabel) {
                ConcatLabel.segment(offset, vec.size()) = vec;
                offset += vec.size();
            }

            Eigen::ArrayXd LowScanOddi(ConcatLabel.size());
            Eigen::ArrayXd LowScanXYi(ConcatLabel.size() * 2);
            for (int j = 0; j < ConcatLabel.size(); ++j) {
                int OddLabel = ConcatLabel(j);
                int XYLabel = 2*OddLabel;
                LowScanOddi(j) = Oddi(OddLabel);
                LowScanXYi(2*j) = XYi(XYLabel);
                LowScanXYi(2*j+1) = XYi(XYLabel+1);
            }
            LowScanXY[i] = LowScanXYi;
            LowScanOdd[i] = LowScanOddi;

        }
    }
}

void FuncCalBound(const Eigen::MatrixXd& Map, Eigen::MatrixXi& IdSelect,Eigen::MatrixXi& IdSelectVar, const ParamStruct& ValParam){
    Eigen::MatrixXd ExpMap = Eigen::MatrixXd::Zero(Map.rows(), Map.cols());
    FuncExpMap(Map, ExpMap);
    Eigen::MatrixXi IntExpMap = ExpMap.cwiseMax(0.0).cwiseMin(1.0).cast<int>();
    ExpMap = IntExpMap.cast<double>();

    int r = (ValParam.SelectKernelSize - 1) / 2;
    int NumSelectCell = round((ValParam.SelectDistance / ValParam.Scale));
    Eigen::MatrixXd MatKernel = Eigen::MatrixXd::Ones(ValParam.SelectKernelSize,ValParam.SelectKernelSize) / (ValParam.SelectKernelSize * ValParam.SelectKernelSize);
    Eigen::MatrixXd ConvMap = Eigen::MatrixXd::Zero(Map.rows(), Map.cols());
    Eigen::MatrixXd Block(2*r+1, 2*r+1);
    for (int i = 0; i < Map.rows(); ++i) {
        for (int j = 0; j < Map.cols(); ++j) {
            if ((i-r+1)>0 && (j-r+1)>0 && (i+r+1)<=ValParam.Sizei && (j+r+1)<=ValParam.Sizej){
                Block = ExpMap.block(i-r, j-r, 2*r+1, 2*r+1);
            }
            else if ((i-r+1)<=0 && (j-r+1)<=0 && (i+r+1)<=ValParam.Sizei && (j+r+1)<=ValParam.Sizej){
                Block = ExpMap.block(i, j, 2*r+1, 2*r+1);
            }
            else if ((i-r+1)>0 && (j-r+1)<=0 && (i+r+1)>ValParam.Sizei && (j+r+1)<=ValParam.Sizej){
                Block = ExpMap.block(i-2*r, j, 2*r+1, 2*r+1);
            }
            else if ((i-r+1)>0 && (j-r+1)>0 && (i+r+1)>ValParam.Sizei && (j+r+1)>ValParam.Sizej){
                Block = ExpMap.block(i-2*r, j-2*r, 2*r+1, 2*r+1);
            }
            else if ((i-r+1)<=0 && (j-r+1)>0 && (i+r+1)<=ValParam.Sizei && (j+r+1)<=ValParam.Sizej){
                Block = ExpMap.block(i, j-r, 2*r+1, 2*r+1);
            }
            else if ((i-r+1)>0 && (j-r+1)>0 && (i+r+1)>ValParam.Sizei && (j+r+1)<=ValParam.Sizej){
                Block = ExpMap.block(i-2*r, j-r, 2*r+1, 2*r+1);
            }
            else if ((i-r+1)>0 && (j-r+1)<=0 && (i+r+1)<=ValParam.Sizei && (j+r+1)<=ValParam.Sizej){
                Block = ExpMap.block(i-r, j, 2*r+1, 2*r+1);
            }
            else if ((i-r+1)>0 && (j-r+1)>0 && (i+r+1)<=ValParam.Sizei && (j+r+1)>ValParam.Sizej){
                Block = ExpMap.block(i-r, j-2*r, 2*r+1, 2*r+1);
            }
            else if ((i-r+1)<=0 && (j-r+1)>0 && (i+r+1)<=ValParam.Sizei && (j+r+1)>ValParam.Sizej){
                Block = ExpMap.block(i, j-2*r, 2*r+1, 2*r+1);
            }
            else {
                std::cout << "Missing some elements, the index is " << i << ", " << j << "\n\n";
            }
            ConvMap(i,j) = (MatKernel.array() * Block.array()).sum();
        }
    }

    Eigen::MatrixXd MapEdge = Eigen::MatrixXd::Zero(ConvMap.rows(), ConvMap.cols());
    MapEdge =(ConvMap.array() > 0.0 && ConvMap.array() < 1.0).select(Eigen::MatrixXd::Ones(ConvMap.rows(), ConvMap.cols()), MapEdge);
    Eigen::ArrayXi IdAll, IdRow, IdCol;
    igl::find(MapEdge.array() == 1.0, IdAll);
    IdCol = (IdAll / MapEdge.rows()).floor();
    IdRow = IdAll - IdCol * MapEdge.rows();


    Eigen::ArrayXi IdRowT = IdRow - NumSelectCell;
    Eigen::ArrayXi IdColL = IdCol - NumSelectCell;

    int InsertPoint = 0;
    Eigen::MatrixXi TemIdSelect((2 * NumSelectCell + 1) * (2 * NumSelectCell + 1) * IdRowT.size(), 2);
    for (int i = 0; i < 2 * NumSelectCell + 1; i++) {
        for (int j = 0; j < 2 * NumSelectCell + 1; j++) {
            InsertPoint = (i*(2*NumSelectCell+1)+j)*IdRowT.size();
            TemIdSelect.block(InsertPoint,0,IdRowT.size(),1) = IdRowT + i;
            TemIdSelect.block(InsertPoint,1,IdRowT.size(),1) = IdColL + j;
        }
    }

    Eigen::MatrixXi UniqIdSelect;
    Eigen::VectorXi IA, IC;
    igl::unique_rows(TemIdSelect, UniqIdSelect, IA, IC);

    Eigen::ArrayXi IdSelRow, IdSelCol;

    IdSelRow = UniqIdSelect.col(0);
    IdSelCol = UniqIdSelect.col(1);

    int IdSize = IdSelRow.rows();

    Eigen::MatrixXi IdSel11(IdSize,2), IdSel12(IdSize,2), IdSel13(IdSize,2), IdSel14(IdSize,2), IdSel15(IdSize,2),
            IdSel21(IdSize,2), IdSel22(IdSize,2), IdSel23(IdSize,2), IdSel24(IdSize,2), IdSel25(IdSize,2),
            IdSel31(IdSize,2), IdSel32(IdSize,2), IdSel34(IdSize,2), IdSel35(IdSize,2),
            IdSel41(IdSize,2), IdSel42(IdSize,2), IdSel43(IdSize,2), IdSel44(IdSize,2), IdSel45(IdSize,2),
            IdSel51(IdSize,2), IdSel52(IdSize,2), IdSel53(IdSize,2), IdSel54(IdSize,2), IdSel55(IdSize,2);

    IdSel11.block(0,0,IdSize,1) = IdSelRow - 2;
    IdSel11.block(0,1,IdSize,1) = IdSelCol - 2;

    IdSel12.block(0,0,IdSize,1) = IdSelRow - 2;
    IdSel12.block(0,1,IdSize,1) = IdSelCol - 1;

    IdSel13.block(0,0,IdSize,1) = IdSelRow - 2;
    IdSel13.block(0,1,IdSize,1) = IdSelCol;

    IdSel14.block(0,0,IdSize,1) = IdSelRow - 2;
    IdSel14.block(0,1,IdSize,1) = IdSelCol + 1;

    IdSel15.block(0,0,IdSize,1) = IdSelRow - 2;
    IdSel15.block(0,1,IdSize,1) = IdSelCol + 2;

    IdSel21.block(0,0,IdSize,1) = IdSelRow - 1;
    IdSel21.block(0,1,IdSize,1) = IdSelCol - 2;

    IdSel22.block(0,0,IdSize,1) = IdSelRow - 1;
    IdSel22.block(0,1,IdSize,1) = IdSelCol - 1;

    IdSel23.block(0,0,IdSize,1) = IdSelRow - 1;
    IdSel23.block(0,1,IdSize,1) = IdSelCol;

    IdSel24.block(0,0,IdSize,1) = IdSelRow - 1;
    IdSel24.block(0,1,IdSize,1) = IdSelCol + 1;

    IdSel25.block(0,0,IdSize,1) = IdSelRow - 1;
    IdSel25.block(0,1,IdSize,1) = IdSelCol + 2;

    IdSel31.block(0,0,IdSize,1) = IdSelRow;
    IdSel31.block(0,1,IdSize,1) = IdSelCol - 2;

    IdSel32.block(0,0,IdSize,1) = IdSelRow;
    IdSel32.block(0,1,IdSize,1) = IdSelCol - 1;

    IdSel34.block(0,0,IdSize,1) = IdSelRow;
    IdSel34.block(0,1,IdSize,1) = IdSelCol + 1;

    IdSel35.block(0,0,IdSize,1) = IdSelRow;
    IdSel35.block(0,1,IdSize,1) = IdSelCol + 2;

    IdSel41.block(0,0,IdSize,1) = IdSelRow + 1;
    IdSel41.block(0,1,IdSize,1) = IdSelCol - 2;

    IdSel42.block(0,0,IdSize,1) = IdSelRow + 1;
    IdSel42.block(0,1,IdSize,1) = IdSelCol - 1;

    IdSel43.block(0,0,IdSize,1) = IdSelRow + 1;
    IdSel43.block(0,1,IdSize,1) = IdSelCol;

    IdSel44.block(0,0,IdSize,1) = IdSelRow + 1;
    IdSel44.block(0,1,IdSize,1) = IdSelCol + 1;

    IdSel45.block(0,0,IdSize,1) = IdSelRow + 1;
    IdSel45.block(0,1,IdSize,1) = IdSelCol + 2;

    IdSel51.block(0,0,IdSize,1) = IdSelRow + 2;
    IdSel51.block(0,1,IdSize,1) = IdSelCol - 2;

    IdSel52.block(0,0,IdSize,1) = IdSelRow + 2;
    IdSel52.block(0,1,IdSize,1) = IdSelCol - 1;

    IdSel53.block(0,0,IdSize,1) = IdSelRow + 2;
    IdSel53.block(0,1,IdSize,1) = IdSelCol;

    IdSel54.block(0,0,IdSize,1) = IdSelRow + 2;
    IdSel54.block(0,1,IdSize,1) = IdSelCol + 1;

    IdSel55.block(0,0,IdSize,1) = IdSelRow + 2;
    IdSel55.block(0,1,IdSize,1) = IdSelCol + 2;


    Eigen::MatrixXi IdVar(IdSel11.rows()*25, 2);
    int offset = 0;
    // Copy data from each array to the new array
    IdVar.block(offset, 0, IdSel11.rows(), 2) = IdSel11;
    offset += IdSel11.rows();

    IdVar.block(offset, 0, IdSel12.rows(), 2) = IdSel12;
    offset += IdSel12.rows();

    IdVar.block(offset, 0, IdSel13.rows(), 2) = IdSel13;
    offset += IdSel13.rows();

    IdVar.block(offset, 0, IdSel14.rows(), 2) = IdSel14;
    offset += IdSel14.rows();

    IdVar.block(offset, 0, IdSel15.rows(), 2) = IdSel15;
    offset += IdSel15.rows();

    IdVar.block(offset, 0, IdSel21.rows(), 2) = IdSel21;
    offset += IdSel21.rows();

    IdVar.block(offset, 0, IdSel22.rows(), 2) = IdSel22;
    offset += IdSel22.rows();

    IdVar.block(offset, 0, IdSel23.rows(), 2) = IdSel23;
    offset += IdSel23.rows();

    IdVar.block(offset, 0, IdSel24.rows(), 2) = IdSel24;
    offset += IdSel24.rows();

    IdVar.block(offset, 0, IdSel25.rows(), 2) = IdSel25;
    offset += IdSel25.rows();

    IdVar.block(offset, 0, IdSel31.rows(), 2) = IdSel31;
    offset += IdSel31.rows();

    IdVar.block(offset, 0, IdSel32.rows(), 2) = IdSel32;
    offset += IdSel32.rows();

    IdVar.block(offset, 0, UniqIdSelect.rows(), 2) = UniqIdSelect;
    offset += UniqIdSelect.rows();

    IdVar.block(offset, 0, IdSel34.rows(), 2) = IdSel34;
    offset += IdSel34.rows();

    IdVar.block(offset, 0, IdSel35.rows(), 2) = IdSel35;
    offset += IdSel35.rows();

    IdVar.block(offset, 0, IdSel41.rows(), 2) = IdSel41;
    offset += IdSel41.rows();

    IdVar.block(offset, 0, IdSel42.rows(), 2) = IdSel42;
    offset += IdSel42.rows();

    IdVar.block(offset, 0, IdSel43.rows(), 2) = IdSel43;
    offset += IdSel43.rows();

    IdVar.block(offset, 0, IdSel44.rows(), 2) = IdSel44;
    offset += IdSel44.rows();

    IdVar.block(offset, 0, IdSel45.rows(), 2) = IdSel45;
    offset += IdSel45.rows();

    IdVar.block(offset, 0, IdSel51.rows(), 2) = IdSel51;
    offset += IdSel51.rows();

    IdVar.block(offset, 0, IdSel52.rows(), 2) = IdSel52;
    offset += IdSel52.rows();

    IdVar.block(offset, 0, IdSel53.rows(), 2) = IdSel53;
    offset += IdSel53.rows();

    IdVar.block(offset, 0, IdSel54.rows(), 2) = IdSel54;
    offset += IdSel54.rows();

    IdVar.block(offset, 0, IdSel55.rows(), 2) = IdSel55;
    offset += IdSel55.rows();

    Eigen::MatrixXi IdVarR = IdVar;
    IdVarR.col(1) = IdVarR.col(1).array() + 1;
    Eigen::MatrixXi IdVarB = IdVar;
    IdVarB.col(0) = IdVarR.col(0).array() + 1;
    Eigen::MatrixXi IdVarBR = IdVar;
    IdVarBR.col(0) = IdVarBR.col(0).array() + 1;
    IdVarBR.col(1) = IdVarBR.col(1).array() + 1;

    Eigen::MatrixXi IdVarAll(IdVar.rows()*4, 2);
    offset = 0;
    IdVarAll.block(offset, 0, IdVar.rows(), 2) = IdVar;
    offset += IdVar.rows();
    IdVarAll.block(offset, 0, IdVarR.rows(), 2) = IdVarR;
    offset += IdVarR.rows();
    IdVarAll.block(offset, 0, IdVarB.rows(), 2) = IdVarB;
    offset += IdVarB.rows();
    IdVarAll.block(offset, 0, IdVarBR.rows(), 2) = IdVarBR;
    offset += IdVarBR.rows();

    Eigen::MatrixXi UniqIdVar;
    igl::unique_rows(IdVarAll, UniqIdVar, IA, IC);
    IdSelect = UniqIdSelect;
    IdSelectVar = UniqIdVar;
}

void FuncExpMap(const Eigen::MatrixXd& Map, Eigen::MatrixXd& ExpMap){
    #pragma omp parallel for
    for (int i = 0; i < Map.rows(); ++i) {
        for (int j = 0; j < Map.cols(); ++j) {
            ExpMap(i,j) = exp(Map(i,j));
        }
    }
}

void FuncSelectScan(const Eigen::MatrixXd& Map, const Eigen::MatrixXd& Pose, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, std::vector<Eigen::ArrayXd>& SelectScanXY, std::vector<Eigen::ArrayXd>& SelectScanOdd, const Eigen::MatrixXi& IdSelect, const ParamStruct& ValParam){
    Eigen::ArrayXi RowIdSelect = IdSelect.col(0);
    Eigen::ArrayXi ColIdSelect = IdSelect.col(1);
    Eigen::SparseMatrix<int> MaskMap(ValParam.Sizei, ValParam.Sizej);
    Eigen::ArrayXi ValOne = Eigen::ArrayXi::Ones(RowIdSelect.size());

    igl::sparse(RowIdSelect,ColIdSelect,ValOne,ValParam.Sizei,ValParam.Sizej,MaskMap);
    Eigen::ArrayXi ArrMask = Eigen::Map<Eigen::ArrayXi>(MaskMap.toDense().data(), MaskMap.toDense().size());
    int NumPose = Pose.size() / 3;
    #pragma omp parallel for
    for (int i = 0; i < NumPose; ++i) {
        Eigen::Rotation2Dd rotation(Pose(i,2));
        Eigen::Matrix2d Ri = rotation.toRotationMatrix();
        Eigen::ArrayXd XYi = ScanXY[i];
        Eigen::ArrayXd Oddi = ScanOdd[i];
        Eigen::Map<Eigen::MatrixXd> MatrixXY(XYi.data(), 2, XYi.size()/2);
        Eigen::Vector2d VectorTrans = Pose.block<>(i,0,1,2).transpose();
        Eigen::MatrixXd Si = (Ri * MatrixXY).colwise() + VectorTrans;
        Eigen::Vector2d Origin(ValParam.OriginX,ValParam.OriginY);
        Eigen::MatrixXi XY3 = ((Si.colwise()-Origin).array() / ValParam.Scale).floor().matrix().cast<int>();
        Eigen::ArrayXi Row = XY3.row(1);
        Eigen::ArrayXi Col = XY3.row(0);
        Eigen::ArrayXi Id = Col * ValParam.Sizei + Row;
        Eigen::ArrayXi Vali = ArrMask(Id);
        Eigen::ArrayXi IdObs;
        igl::find(Vali==1, IdObs);
        Eigen::ArrayXd ScanXYSelecti(IdObs.size()*2);
        Eigen::ArrayXd ScanOddSelecti(IdObs.size());
        for (int j = 0; j < IdObs.size(); ++j) {
            int OddId = IdObs(j);
            int XYId = 2*OddId;
            ScanOddSelecti(j) = Oddi(OddId);
            ScanXYSelecti(2*j) = XYi(XYId);
            ScanXYSelecti(2*j+1) = XYi(XYId+1);
        }
        SelectScanXY[i] = ScanXYSelecti;
        SelectScanOdd[i] = ScanOddSelecti;
    }
}

void FuncInitialSelectMap(Eigen::MatrixXd& Map, Eigen::MatrixXd& N, Eigen::MatrixXd& Pose, std::vector<Eigen::ArrayXd>& SelectScanXY, std::vector<Eigen::ArrayXd>& SelectScanOdd, ParamStruct& ValParam){
    int NumPose = Pose.size() / 3;

    Map = Eigen::MatrixXd::Zero(ValParam.Sizei, ValParam.Sizej);
    N = Eigen::MatrixXd::Zero(ValParam.Sizei, ValParam.Sizej);
    #pragma omp parallel for
    for (int i = 0; i < NumPose; ++i) {
        Eigen::Rotation2Dd rotation(Pose(i, 2));
        Eigen::Matrix2d Ri = rotation.toRotationMatrix();
        Eigen::ArrayXd XYi = SelectScanXY[i];
        Eigen::ArrayXd Oddi = SelectScanOdd[i];
        Eigen::Map<Eigen::MatrixXd> MatrixXY(XYi.data(), 2, XYi.size() / 2);
        Eigen::Vector2d VectorTrans = Pose.block<>(i, 0, 1, 2).transpose();
        Eigen::MatrixXd Si = (Ri * MatrixXY).colwise() + VectorTrans;
        Eigen::Vector2d Origin(ValParam.OriginX, ValParam.OriginY);
        Eigen::MatrixXd XY3 = ((Si.colwise() - Origin).array() / ValParam.Scale).matrix();
        Eigen::ArrayXd ContinuousRow = XY3.row(1);
        Eigen::ArrayXd ContinuousCol = XY3.row(0);
        Eigen::ArrayXd Row = ContinuousRow.floor();
        Eigen::ArrayXd Col = ContinuousCol.floor();

        Eigen::ArrayXd a0 = ContinuousCol - Col;
        Eigen::ArrayXd a1 = Col + 1 - ContinuousCol;
        Eigen::ArrayXd b0 = ContinuousRow - Row;
        Eigen::ArrayXd b1 = Row + 1 - ContinuousRow;
        Eigen::MatrixXd Interp(Col.size(),4);
        Interp.col(0) = a1 * b1;
        Interp.col(1) = a0 * b1;
        Interp.col(2) = a1 * b0;
        Interp.col(3) = a0 * b0;
        Eigen::MatrixXd RepmatOddi = Oddi.replicate<1,4>();
        Eigen::MatrixXd FourGridVal = Interp.array() * RepmatOddi.array();

        Eigen::ArrayXd ArrayFourGridVal = (FourGridVal.reshaped(1,FourGridVal.size())).row(0);
        Eigen::ArrayXd ArrayFourNVal = (Interp.reshaped(1, Interp.size())).row(0);

        Eigen::ArrayXd FourGridRow(FourGridVal.size(),1);
        FourGridRow << Row, Row, Row+1, Row+1;
        Eigen::ArrayXd FourGridCol(FourGridVal.size(),1);
        FourGridCol << Col, Col+1, Col, Col+1;

        Eigen::SparseMatrix<double> TemGlobal(ValParam.Sizei, ValParam.Sizej), TemMapN(ValParam.Sizei, ValParam.Sizej);

        igl::sparse(FourGridRow,FourGridCol,ArrayFourGridVal,ValParam.Sizei,ValParam.Sizej,TemGlobal);
        igl::sparse(FourGridRow,FourGridCol,ArrayFourNVal,ValParam.Sizei,ValParam.Sizej,TemMapN);

        Map = Map + TemGlobal;
        N = N + TemMapN;
    }
}

void FuncSelectMapConst(const Eigen::MatrixXd& SelectMap, const Eigen::MatrixXi& IdSelectVar, Eigen::SparseMatrix<double>& HH, const ParamStruct& ValParam){
    Eigen::ArrayXi RowIdSelectVar = IdSelectVar.col(0);
    Eigen::ArrayXi ColIdSelectVar = IdSelectVar.col(1);

    Eigen::ArrayXi ColRight = ColIdSelectVar + 1;
    Eigen::ArrayXi RowBelow = RowIdSelectVar + 1;
    // Sort the order to variables order
    Eigen::MatrixXi OneIdSelectVar(RowIdSelectVar.size(),1);
    Eigen::ArrayXi ArraySortSelectId = RowIdSelectVar * ValParam.Sizej + ColIdSelectVar;
    std::sort(ArraySortSelectId.data(), ArraySortSelectId.data() + ArraySortSelectId.size());
    OneIdSelectVar.col(0) = ArraySortSelectId;
    Eigen::MatrixXi IdRightVar (RowIdSelectVar.size(),1);
    Eigen::ArrayXi ArraySortIdRightVar = RowIdSelectVar * ValParam.Sizej + ColRight;
    std::sort(ArraySortIdRightVar.data(), ArraySortIdRightVar.data() + ArraySortIdRightVar.size());
    IdRightVar.col(0) = ArraySortIdRightVar;
    Eigen::MatrixXi IdBelowVar (RowIdSelectVar.size(),1);
    Eigen::ArrayXi ArraySortIdBelowVar = RowBelow * ValParam.Sizej + ColIdSelectVar;
    std::sort(ArraySortIdBelowVar.data(), ArraySortIdBelowVar.data() + ArraySortIdBelowVar.size());
    IdBelowVar.col(0) = ArraySortIdBelowVar;

    Eigen::ArrayXi BoolFindRight, BoolFindBelow, FindRightPosition, FindBelowPosition;

    igl::ismember(IdRightVar, OneIdSelectVar, BoolFindRight, FindRightPosition);
    igl::ismember(IdBelowVar, OneIdSelectVar, BoolFindBelow, FindBelowPosition);

    Eigen::ArrayXi CheckRight, CheckBelow;
    igl::find(BoolFindRight==1, CheckRight);
    igl::find(BoolFindBelow==1, CheckBelow);
    int nR = CheckRight.size();
    int nB = CheckBelow.size();

    Eigen::ArrayXi ID1 = Eigen::ArrayXi::Ones(2*(nR+nB));
    Eigen::ArrayXi ID2 = ID1;
    Eigen::ArrayXi Val = ID1;
    int nCnt = 0;
    int ij0, ij1, ij2, TemBelowCell;
    Eigen::ArrayXi Arrij2;
    for (int i = 0; i < OneIdSelectVar.size(); ++i) {
        ij0 = i;
        if (BoolFindRight(i)==1){
            nCnt++;
            ij1 = i + 1;
            ID1.segment<2>(2 * nCnt - 2) = (nCnt - 1) * ID1.segment<2>(2 * nCnt - 2);
            ID2.segment<2>(2 * nCnt - 2) << ij0, ij1;
            Val.segment<2>(2 * nCnt - 2) << 1, -1;
        }

        if (BoolFindBelow(i)==1){
            nCnt++;
            TemBelowCell = RowBelow(i) * ValParam.Sizej + ColIdSelectVar(i);
            igl::find(ArraySortSelectId == TemBelowCell, Arrij2);
            ij2 = Arrij2(0);
            ID1.segment<2>(2 * nCnt - 2) = (nCnt - 1) * ID1.segment<2>(2 * nCnt - 2);
            ID2.segment<2>(2 * nCnt - 2) << ij0, ij2;
            Val.segment<2>(2 * nCnt - 2) << 1, -1;
        }
    }

    int MaxID1 = ID1.maxCoeff();
    int MaxID2 = ID2.maxCoeff();
    Eigen::SparseMatrix<double> J(MaxID1, MaxID2);
    igl::sparse(ID1, ID2, Val, MaxID1+1, MaxID2+1, J);
    HH = J.transpose() * J;
}

void FuncSmoothSelectN2(Eigen::MatrixXd& SelectN, const Eigen::SparseMatrix<double>& WeightHHSelect, const Eigen::MatrixXi IdSelectVar, const ParamStruct& ValParam){
    Eigen::SparseMatrix<double> SparseN = SelectN.sparseView();
    Eigen::VectorXi I, J;
    Eigen::VectorXd Val;
    igl::find(SparseN, I, J ,Val);
    int ni = I.size();
    Eigen::VectorXi ID1(ni);
    ID1.setLinSpaced(ni, 0, ni - 1);
    Eigen::VectorXi ID2(ni);
    ID2 = ValParam.Sizej * I  + J;
    Eigen::VectorXd VecOnes = Eigen::VectorXd::Ones(ni);
    Eigen::SparseMatrix<double> A1;
    igl::sparse(ID1,ID2,VecOnes,ni,ValParam.Sizei * ValParam.Sizej, A1);

    Eigen::ArrayXi RowIdSelectVar = IdSelectVar.col(0);
    Eigen::ArrayXi ColIdSelectVar = IdSelectVar.col(1);
    Eigen::ArrayXi ArraySortSelectId = RowIdSelectVar * ValParam.Sizej + ColIdSelectVar;
    std::sort(ArraySortSelectId.data(), ArraySortSelectId.data() + ArraySortSelectId.size());

    Eigen::SparseMatrix<double> A1Select(A1.rows(),ArraySortSelectId.size());

    for (int j = 0; j < ArraySortSelectId.size(); ++j)
    {
        A1Select.col(j) = A1.col(ArraySortSelectId(j));
    }

    Eigen::SparseMatrix<double> II = A1Select.transpose() * A1Select + WeightHHSelect;
    Eigen::SparseMatrix<double> EE = (A1Select.transpose() * Val).sparseView();
    II.makeCompressed();
    Eigen::initParallel();
    Eigen::ConjugateGradient <Eigen::SparseMatrix<double>, Eigen::Upper|Eigen::Lower> solver;
    int MaxNum = ValParam.SolverSecondMaxIter;
    solver.setMaxIterations(MaxNum);
    solver.setTolerance(ValParam.SolverSecondTolerance);
    solver.compute(II);
    Eigen::VectorXd SelectDeltaN = solver.solve(EE);

//    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver;
//    II.makeCompressed();
//    solver.compute(II);
//    Eigen::VectorXd SelectDeltaN = solver.solve(EE);

    Eigen::ArrayXi ColumnId = Eigen::ArrayXi::Zero(ArraySortSelectId.size());
    Eigen::SparseMatrix<double> DeltaN(ValParam.Sizei*ValParam.Sizej,1);

    igl::sparse(ArraySortSelectId,ColumnId,SelectDeltaN,ValParam.Sizei*ValParam.Sizej,1,DeltaN);
    SelectN = DeltaN.toDense().reshaped(ValParam.Sizej,ValParam.Sizei).transpose();
}

std::tuple<Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::ArrayXd,Eigen::ArrayXd,double,double> FuncDiffSelectJacobian(const Eigen::MatrixXd& SelectMap, const Eigen::MatrixXd& SelectN, const Eigen::MatrixXd& Pose, const Eigen::MatrixXd& Odom, const std::vector<Eigen::ArrayXd>& SelectScanXY, const std::vector<Eigen::ArrayXd>& SelectScanOdd, const Eigen::MatrixXi& IdSelectVar, const ParamStruct& ValParam){
    int NumPose = Pose.size()/3;
    // Calculate the gradient of map
    double h = 1.0;
    Eigen::MatrixXd G = FuncGradient(SelectMap,h);
    Eigen::MatrixXd Gv = G.block(0, 0, SelectMap.rows(), SelectMap.cols());
    Eigen::MatrixXd Gu = G.block(0, SelectMap.cols(), SelectMap.rows(), SelectMap.cols());

    Eigen::ArrayXi RowIdSelectVar = IdSelectVar.col(0);
    Eigen::ArrayXi ColIdSelectVar = IdSelectVar.col(1);
    Eigen::MatrixXi OneIdSelectVar(RowIdSelectVar.size(),1);
    Eigen::ArrayXi ArraySelectId = RowIdSelectVar * ValParam.Sizej + ColIdSelectVar;
    std::sort(ArraySelectId.data(), ArraySelectId.data() + ArraySelectId.size());
    OneIdSelectVar.col(0) = ArraySelectId;

    int nPts = 0;

    Eigen::ArrayXi JPID1;
    Eigen::ArrayXi JPID2;
    Eigen::ArrayXd JPVal;

    Eigen::ArrayXi JDID1;
    Eigen::ArrayXi JDID2;
    Eigen::ArrayXd JDVal;

    Eigen::ArrayXi JOID1;
    Eigen::ArrayXi JOID2;
    Eigen::ArrayXd JOVal;

    Eigen::ArrayXd ErrorS;
    Eigen::ArrayXd ErrorO;

    for (int i = 0; i < NumPose; ++i) {
        Eigen::Rotation2Dd rotation(Pose(i,2));
        Eigen::Matrix2d Ri = rotation.toRotationMatrix();
        Eigen::ArrayXd XYi = SelectScanXY[i];
        Eigen::ArrayXd Oddi = SelectScanOdd[i];
        Eigen::MatrixXd MatrixXY = Eigen::Map<Eigen::MatrixXd>(XYi.data(), 2, XYi.size()/2);
        Eigen::Vector2d VectorTrans = Pose.block<>(i,0,1,2).transpose();
        Eigen::MatrixXd XY2 = (Ri * MatrixXY).colwise() + VectorTrans;
        Eigen::Vector2d Origin(ValParam.OriginX,ValParam.OriginY);
        Eigen::MatrixXd XY3 = ((XY2.colwise()-Origin).array() / ValParam.Scale).matrix();

        //check if some points out of selected variables
        Eigen::MatrixXi UV =  XY3.array().floor().cast<int>().matrix();

        Eigen::ArrayXd u = XY3.row(0);
        Eigen::ArrayXd v = XY3.row(1);

        Eigen::ArrayXd u1 = UV.row(0).cast<double>();
        Eigen::ArrayXd v1 = UV.row(1).cast<double>();

        Eigen::ArrayXd aa = ValParam.Sizej * v1 + u1;
        Eigen::ArrayXd bb = ValParam.Sizej * (v1 + 1) + u1;
        Eigen::ArrayXd cc = ValParam.Sizej * v1 + u1 + 1;
        Eigen::ArrayXd dd = ValParam.Sizej * (v1 + 1) + u1 +1;

        Eigen::ArrayXi pa, pb, pc, pd;
        Eigen::ArrayXi Boola, Boolb, Boolc, Boold;
        Eigen::MatrixXi Mata(aa.size(),1), Matb(bb.size(),1), Matc(aa.size(),1), Matd(bb.size(),1);
        Mata.col(0) = aa.cast<int>();
        Matb.col(0) = bb.cast<int>();
        Matc.col(0) = cc.cast<int>();
        Matd.col(0) = dd.cast<int>();
        igl::ismember(Mata, OneIdSelectVar, Boola, pa);
        igl::ismember(Matb, OneIdSelectVar, Boolb, pb);
        igl::ismember(Matc, OneIdSelectVar, Boolc, pc);
        igl::ismember(Matd, OneIdSelectVar, Boold, pd);

        Eigen::ArrayXi BoolOut = Boola * Boolb * Boolc * Boold;
        Eigen::ArrayXi OutId;
        igl::find(BoolOut==0, OutId);

        if (OutId.size()!=0){
            RemoveArrayIndex(Oddi, OutId);
            RemoveArrayIndex(pa, OutId);
            RemoveArrayIndex(pb, OutId);
            RemoveColumn(XY3, OutId);
            RemoveColumn(MatrixXY, OutId);
        }
        pc = pa + 1;
        pd = pb + 1;


        // Interpolation
        Eigen::ArrayXd MapInterpXY3 = FuncBilinearInterpolation(SelectMap, XY3.transpose());
        Eigen::ArrayXd NInterpXY3 = FuncBilinearInterpolation(SelectN, XY3.transpose());


        // Calculation of Error w.r.t. Observations Term
        Eigen::ArrayXd Ei = MapInterpXY3.cwiseQuotient(NInterpXY3) - Oddi;

        ErrorS.conservativeResize(ErrorS.size()+Ei.size());
        ErrorS.tail(Ei.size()) = Ei;

        Eigen::ArrayXd GuInterpXY3 = FuncBilinearInterpolation(Gu, XY3.transpose());
        Eigen::ArrayXd GvInterpXY3 = FuncBilinearInterpolation(Gv, XY3.transpose());

        // Calculation of Jacobian of Observation Terms w.r.t. Poses
        Eigen::MatrixXd dMdXY3(2,NInterpXY3.size());
        dMdXY3.row(0) = GuInterpXY3 / NInterpXY3;
        dMdXY3.row(1) = GvInterpXY3 / NInterpXY3;
        Eigen::Matrix2d dR = FuncdR2D(Pose(i,2));
        Eigen::MatrixXd dXY3dR = dR.transpose() * MatrixXY / ValParam.Scale;
        Eigen::MatrixXd dMdR = (dMdXY3.array() * dXY3dR.array()).matrix().colwise().sum();
        Eigen::MatrixXd dMdT = dMdXY3 / ValParam.Scale;
        Eigen::MatrixXd dMdP(3,dMdR.size());
        dMdP << dMdT, dMdR;

        int nPtsi = Oddi.size();
        Eigen::ArrayXi IDi = Eigen::ArrayXi::LinSpaced(nPtsi, nPts, nPts + nPtsi -1);
        nPts = nPts+nPtsi;
        Eigen::MatrixXi dEdPID1 = IDi.replicate<1,3>();

        Eigen::ArrayXi IDCol = Eigen::ArrayXi::LinSpaced(3, 3*i, 3*(i+1)-1);
        Eigen::MatrixXi dEdPID2 = IDCol.replicate(1, nPtsi).transpose();

        Eigen::ArrayXi JPID1i = Eigen::Map<Eigen::ArrayXi>(dEdPID1.data(), dEdPID1.size());
        Eigen::ArrayXi JPID2i = Eigen::Map<Eigen::ArrayXi>(dEdPID2.data(), dEdPID2.size());
        Eigen::MatrixXd dMdPTranpose = dMdP.transpose();
        Eigen::ArrayXd JPVali = Eigen::Map<Eigen::ArrayXd>(dMdPTranpose.data(), dMdPTranpose.size());

        JPID1.conservativeResize(JPID1.size()+JPID1i.size());
        JPID1.tail(JPID1i.size()) = JPID1i;
        JPID2.conservativeResize(JPID2.size()+JPID2i.size());
        JPID2.tail(JPID2i.size()) = JPID2i;
        JPVal.conservativeResize(JPVal.size()+JPVali.size());
        JPVal.tail(JPVali.size()) = JPVali;


        // Calculation of Jacobian of Observation Terms w.r.t. Map
        // delete UV, u, v, u1, v1;
        UV =  XY3.array().floor().cast<int>().matrix();

        u = XY3.row(0);
        v = XY3.row(1);
        u1 = UV.row(0).cast<double>();
        v1 = UV.row(1).cast<double>();

        Eigen::ArrayXd a = (v1+1-v) * (u1+1-u) / NInterpXY3;
        Eigen::ArrayXd b = (v-v1) * (u1+1-u) / NInterpXY3;
        Eigen::ArrayXd c = (v1+1-v) * (u-u1) / NInterpXY3;
        Eigen::ArrayXd d = (v-v1) * (u-u1) / NInterpXY3;

        Eigen::MatrixXd dEdM(4,u.size());
        dEdM << Eigen::Map<Eigen::ArrayXXd>(a.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXd>(b.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXd>(c.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXd>(d.data(), 1, u.size());

        Eigen::MatrixXi dEdMID2(4,u.size());

        dEdMID2 << Eigen::Map<Eigen::ArrayXXi>(pa.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXi>(pb.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXi>(pc.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXi>(pd.data(), 1, u.size());
        Eigen::MatrixXi dEdMID1 = IDi.replicate<1,4>();
        Eigen::ArrayXi JDID1i = Eigen::Map<Eigen::ArrayXi>(dEdMID1.data(), dEdMID1.size());


        Eigen::MatrixXi dEdMID2Tranp = dEdMID2.transpose().cast<int>();
        Eigen::ArrayXi JDID2i = Eigen::Map<Eigen::ArrayXi>(dEdMID2Tranp.data(), dEdMID2Tranp.size());
        Eigen::MatrixXd dEdMTranp = dEdM.transpose();
        Eigen::ArrayXd JDVali = Eigen::Map<Eigen::ArrayXd>(dEdMTranp.data(), dEdMTranp.size());

        JDID1.conservativeResize(JDID1.size()+JDID1i.size());
        JDID1.tail(JDID1i.size()) = JDID1i;
        JDID2.conservativeResize(JDID2.size()+JDID2i.size());
        JDID2.tail(JDID2i.size()) = JDID2i;
        JDVal.conservativeResize(JDVal.size()+JDVali.size());
        JDVal.tail(JDVali.size()) = JDVali;
        // Calculate Jacobian w.r.t. the odometry term
        if (i < NumPose-1){
            int Cnti = 3 * i;
            Eigen::ArrayXd P1 =  Pose.row(i);
            Eigen::ArrayXd P2 =  Pose.row(i+1);
            Eigen::MatrixXd dT = Ri.transpose() * (P2.head(2) - P1.head(2)).matrix();
            double dPhi = P2(2)-P1(2);
            while (dPhi> M_PI || dPhi< - M_PI){
                dPhi = FuncWrap(dPhi);
            }
            Eigen::MatrixXd EOi(3,1);
            EOi.block(0,0,2,1) = dT;
            EOi(2,0) = dPhi;
            EOi = EOi - Odom.row(i+1).transpose();
            while (EOi(2,0)> M_PI || EOi(2,0)< - M_PI){
                EOi(2,0) = FuncWrap(EOi(2,0));
            }
            ErrorO.conservativeResize(ErrorO.size()+EOi.size());
            ErrorO.tail(EOi.size()) = EOi;

            Eigen::Matrix2d dTdT1 = -Ri.transpose();
            Eigen::Matrix2d dTdT2 = Ri.transpose();
            Eigen::MatrixXd dTdPhi1 = dR * (P2.head(2) - P1.head(2)).matrix();
            double dPhid1 = -1.0;
            double dPhid2 = 1.0;
            Eigen::ArrayXi aaa = Eigen::ArrayXi::LinSpaced(3, 3*i, 3*i+2);
            Eigen::ArrayXi bbb = Eigen::ArrayXi::LinSpaced(3, 3*(i+1), 3*(i+1)+2);
            Eigen::ArrayXi JOID1i(12);
            JOID1i << Cnti,Cnti,Cnti+1,Cnti+1,Cnti,Cnti,Cnti+1,Cnti+1,Cnti,Cnti+1,Cnti+2,Cnti+2;
            Eigen::ArrayXi JOID2i(12);
            JOID2i << aaa.head(2), aaa.head(2), bbb.head(2), bbb.head(2), aaa(2), aaa(2), aaa(2), bbb(2);

            Eigen::ArrayXd JOVali(12);
            JOVali.head(2) = dTdT1.row(0);
            JOVali.segment(2,2) = dTdT1.row(1);
            JOVali.segment(4,2) = dTdT2.row(0);
            JOVali.segment(6,2) = dTdT2.row(1);
            JOVali(8) = dTdPhi1(0,0);
            JOVali(9) = dTdPhi1(1,0);
            JOVali(10) = dPhid1;
            JOVali(11) = dPhid2;

            JOID1.conservativeResize(JOID1.size()+JOID1i.size());
            JOID1.tail(JOID1i.size()) = JOID1i;
            JOID2.conservativeResize(JOID2.size()+JOID2i.size());
            JOID2.tail(JOID2i.size()) = JOID2i;
            JOVal.conservativeResize(JOVal.size()+JOVali.size());
            JOVal.tail(JOVali.size()) = JOVali;
        }
    }

    Eigen::SparseMatrix<double> JP(ErrorS.size(),3*NumPose), JD(ErrorS.size(),ValParam.Sizei*ValParam.Sizej), JO(3*(NumPose-1),3*NumPose);
    std::thread t1([&]() {
        igl::sparse(JPID1, JPID2, JPVal, ErrorS.size(), 3 * NumPose, JP);
    });
    std::thread t2([&]() {
        igl::sparse(JDID1, JDID2, JDVal, ErrorS.size(), ArraySelectId.size(), JD);
    });
    std::thread t3([&]() {
        igl::sparse(JOID1, JOID2, JOVal, 3 * (NumPose - 1), 3 * NumPose, JO);
    });
    t1.join();
    t2.join();
    t3.join();

    JP.makeCompressed();
    JD.makeCompressed();
    JO.makeCompressed();

    auto [IS,IO] = FuncGetI(Odom,ErrorS);
    IO.makeCompressed();
    IS.makeCompressed();

    Eigen::MatrixXd MatErrorS = Eigen::MatrixXd::Map(ErrorS.data(), ErrorS.size(), 1);
    Eigen::MatrixXd MatErrorO = Eigen::MatrixXd::Map(ErrorO.data(), ErrorO.size(), 1);

    double SumErrorS = (MatErrorS.transpose() * IS * MatErrorS)(0,0);
    double SumErrorO = (ValParam.WeightO * MatErrorO.transpose() * IO * MatErrorO)(0,0);
    double SumError = SumErrorS + SumErrorO;
    double MeanError = SumErrorS/ErrorS.size() + SumErrorO/ErrorO.size();
    return std::make_tuple(JP,JD,JO,IS,IO,ErrorS,ErrorO,SumError,MeanError);
}


void RemoveColumn(Eigen::MatrixXd& matrix, const Eigen::ArrayXi& B) {

    std::vector<int> sorted_indices(B.data(), B.data() + B.size());
    std::sort(sorted_indices.begin(), sorted_indices.end(), std::greater<int>());

    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols();
    unsigned int numColsToRemove = B.size();

    for (unsigned int i = 0; i < numColsToRemove; ++i) {
        unsigned int colToRemove = sorted_indices[i];
        matrix.block(0, colToRemove, numRows, numCols - colToRemove - 1) = matrix.block(0, colToRemove + 1, numRows, numCols - colToRemove - 1);
        numCols--;
    }

    matrix.conservativeResize(numRows,numCols);
}

void RemoveArrayIndex(Eigen::ArrayXd& A, const Eigen::ArrayXi& B)
{
    std::vector<int> sorted_indices(B.data(), B.data() + B.size());
    std::vector<double> v(A.data(), A.data() + A.size());
    std::sort(sorted_indices.begin(), sorted_indices.end(), std::greater<int>());
    for (int i = 0; i < sorted_indices.size(); i++) {
        v.erase(v.begin() + sorted_indices[i]);
    }
    Eigen::Map<Eigen::ArrayXd> C(v.data(), v.size());
    A = C;
}

void RemoveArrayIndex(Eigen::ArrayXi& A, const Eigen::ArrayXi& B)
{
    std::vector<int> sorted_indices(B.data(), B.data() + B.size());
    std::vector<int> v(A.data(), A.data() + A.size());
    std::sort(sorted_indices.begin(), sorted_indices.end(), std::greater<int>());
    for (int i = B.size() - 1; i >= 0; i--) {
        v.erase(v.begin() + B[i]);
    }
    Eigen::Map<Eigen::ArrayXi> C(v.data(), v.size());
    A = C;
}


std::tuple<Eigen::VectorXd, Eigen::VectorXd, double, double, double, double> FuncSelectMapDelta(const Eigen::MatrixXd& SelectMap, const Eigen::MatrixXd& Odom, const Eigen::SparseMatrix<double>& JP, const Eigen::SparseMatrix<double>& JD, const Eigen::SparseMatrix<double>& JO, const Eigen::ArrayXd& ErrorS, const Eigen::ArrayXd& ErrorO, const Eigen::SparseMatrix<double>& IS, const Eigen::SparseMatrix<double>& IO, const Eigen::SparseMatrix<double>& WeightHH, const Eigen::MatrixXi& IdSelectVar, const ParamStruct& ValParam){

    Eigen::SparseMatrix<double> JPSlice = JP.block(0, 3, JP.rows(), JP.cols()-3);
    Eigen::SparseMatrix<double> JOSlice = JO.block(0, 3, JO.rows(), JO.cols()-3);
    Eigen::SparseMatrix<double,Eigen::RowMajor> JDRow(JD);

    JPSlice.makeCompressed();
    JOSlice.makeCompressed();
    JDRow.makeCompressed();

    Eigen::SparseMatrix<double> U = JPSlice.transpose() * JPSlice + ValParam.WeightO * JOSlice.transpose() * IO * JOSlice;

    Eigen::SparseMatrix<double> V = JDRow.transpose() * JDRow + WeightHH;
    Eigen::SparseMatrix<double> W = JPSlice.transpose() * JDRow;

    Eigen::VectorXd VecErrorS = Eigen::VectorXd::Map(ErrorS.data(), ErrorS.size());
    Eigen::VectorXd VecErrorO = Eigen::VectorXd::Map(ErrorO.data(), ErrorO.size());

    Eigen::VectorXd EP = -JPSlice.transpose() * VecErrorS - ValParam.WeightO * JOSlice.transpose() * IO * VecErrorO;
    Eigen::VectorXd ED = -JDRow.transpose() * VecErrorS;

    Eigen::ArrayXi RowIdSelectVar = IdSelectVar.col(0);
    Eigen::ArrayXi ColIdSelectVar = IdSelectVar.col(1);
    // Sort the order to variables order
    Eigen::ArrayXi ArraySortSelectId = RowIdSelectVar * ValParam.Sizej + ColIdSelectVar;
    std::sort(ArraySortSelectId.data(), ArraySortSelectId.data() + ArraySortSelectId.size());

    Eigen::ArrayXd SortSelectMap = SelectMap.transpose().reshaped(ValParam.Sizei*ValParam.Sizej,1);
    Eigen::VectorXd XH0 = SortSelectMap(ArraySortSelectId);

    Eigen::VectorXd EH = -WeightHH * XH0;
    Eigen::VectorXd EDEH = ED + EH;

    Eigen::SparseMatrix<double> UW = igl::cat(2, U, W);
    Eigen::SparseMatrix<double> WT = W.transpose();
    Eigen::SparseMatrix<double> WV = igl::cat(2, WT, V);
    Eigen::SparseMatrix<double> II = igl::cat(1, UW, WV);
    Eigen::VectorXd EE = igl::cat(1,EP,EDEH);

    II.makeCompressed();
    Eigen::initParallel();
    Eigen::ConjugateGradient <Eigen::SparseMatrix<double>, Eigen::Upper|Eigen::Lower> solver;
    int MaxNum = floor(sqrt(II.rows()));
    if (MaxNum > ValParam.SolverSecondMaxIter){MaxNum = ValParam.SolverSecondMaxIter;}
    solver.setMaxIterations(MaxNum);
    solver.setTolerance(ValParam.SolverSecondTolerance);
    solver.compute(II);
    Eigen::VectorXd Delta = solver.solve(EE);

    Eigen::VectorXd DeltaP = Delta.head(JPSlice.cols());
    Eigen::VectorXd DeltaD = Delta.tail(Delta.size() - JPSlice.cols());

    double SumDelta = Delta.transpose() * Delta;
    double MeanDelta = SumDelta / Delta.size();
    double SumDeltaP = DeltaP.transpose() * DeltaP;
    double MeanDeltaP = SumDeltaP / DeltaP.size();

    return std::make_tuple(DeltaP,DeltaD,SumDelta,MeanDelta,SumDeltaP,MeanDeltaP);
}

void FuncSelectMapUpdate(Eigen::MatrixXd& SelectMap, Eigen::MatrixXd& Pose, const Eigen::VectorXd& DeltaP, const Eigen::VectorXd& DeltaD, const Eigen::MatrixXi& IdSelectVar){
    Eigen::MatrixXd DeltaP2 = Eigen::Map<const Eigen::MatrixXd>(DeltaP.data(), 3, DeltaP.size()/3).transpose();
    Pose.block(1, 0, Pose.rows()-1, Pose.cols()) += DeltaP2;
    int Size_i = SelectMap.rows();
    int Size_j = SelectMap.cols();

    Eigen::ArrayXi RowIdSelectVar = IdSelectVar.col(0);
    Eigen::ArrayXi ColIdSelectVar = IdSelectVar.col(1);
    Eigen::ArrayXi ArraySelectId = RowIdSelectVar * Size_j + ColIdSelectVar;
    std::sort(ArraySelectId.data(), ArraySelectId.data() + ArraySelectId.size());

    Eigen::ArrayXd ArrSelectMap = SelectMap.transpose().reshaped(Size_i*Size_j,1);

    ArrSelectMap(ArraySelectId) += DeltaD.array();
    SelectMap = ArrSelectMap.reshaped(Size_j, Size_i).transpose();

//    Eigen::MatrixXd ExpMap = Eigen::MatrixXd::Zero(SelectMap.rows(), SelectMap.cols());
//    FuncExpMap(SelectMap, ExpMap);
}

void FuncUpdateSelectN(Eigen::MatrixXd& SelectN, const Eigen::MatrixXd& Pose, const std::vector<Eigen::ArrayXd>& SelectScanXY, const ParamStruct& ValParam){
    int NumPose = Pose.size() / 3;
    #pragma omp parallel for
    for (int i = 0; i < NumPose; ++i) {
        Eigen::Rotation2Dd rotation(Pose(i, 2));
        Eigen::Matrix2d Ri = rotation.toRotationMatrix();
        Eigen::ArrayXd XYi = SelectScanXY[i];
        Eigen::Map<Eigen::MatrixXd> MatrixXY(XYi.data(), 2, XYi.size() / 2);
        Eigen::Vector2d VectorTrans = Pose.block<>(i, 0, 1, 2).transpose();
        Eigen::MatrixXd Si = (Ri * MatrixXY).colwise() + VectorTrans;
        Eigen::Vector2d Origin(ValParam.OriginX, ValParam.OriginY);
        Eigen::MatrixXd XY3 = ((Si.colwise() - Origin).array() / ValParam.Scale).matrix();
        Eigen::ArrayXd ContinuousRow = XY3.row(1);
        Eigen::ArrayXd ContinuousCol = XY3.row(0);
        Eigen::ArrayXd Row = ContinuousRow.floor();
        Eigen::ArrayXd Col = ContinuousCol.floor();

        Eigen::ArrayXd a0 = ContinuousCol - Col;
        Eigen::ArrayXd a1 = Col + 1 - ContinuousCol;
        Eigen::ArrayXd b0 = ContinuousRow - Row;
        Eigen::ArrayXd b1 = Row + 1 - ContinuousRow;
        Eigen::MatrixXd Interp(Col.size(),4);
        Interp.col(0) = a1 * b1;
        Interp.col(1) = a0 * b1;
        Interp.col(2) = a1 * b0;
        Interp.col(3) = a0 * b0;
        Eigen::ArrayXd ArrayFourNVal = (Interp.reshaped(1, Interp.size())).row(0);

        Eigen::ArrayXd FourGridRow(ArrayFourNVal.size(),1);
        FourGridRow << Row, Row, Row+1, Row+1;
        Eigen::ArrayXd FourGridCol(ArrayFourNVal.size(),1);
        FourGridCol << Col, Col+1, Col, Col+1;

        Eigen::MatrixXd TemGlobal = Eigen::MatrixXd::Zero(ValParam.Sizei, ValParam.Sizej);
        Eigen::MatrixXd TemMapN = Eigen::MatrixXd::Zero(ValParam.Sizei, ValParam.Sizej);

        for (int j = 0; j < ArrayFourNVal.size(); ++j) {
            int tem_col = FourGridCol(j);
            int tem_row = FourGridRow(j);
            double tem_N = ArrayFourNVal(j);
            TemMapN(tem_row, tem_col) = TemMapN(tem_row, tem_col) + tem_N;
        }
        SelectN += TemMapN;
    }
}


void deleteColumns(Eigen::SparseMatrix<double>& matrix, std::vector<int>& colsToDelete)
{
    // Sort the columns to delete in descending order, so that we can delete them one by one
    std::sort(colsToDelete.begin(), colsToDelete.end(), std::greater<int>());

    // Iterate over the columns to delete
    for (int j : colsToDelete)
    {
        // Find the start and end indices of the j-th column
        int start = matrix.outerIndexPtr()[j];
        int end = matrix.outerIndexPtr()[j + 1];

        // Clear the values of the j-th column
        std::fill(matrix.valuePtr() + start, matrix.valuePtr() + end, 0);

        // Update the inner indices and values of the matrix
        for (int k = start; k < end; ++k)
        {
            int i = matrix.innerIndexPtr()[k];
            matrix.innerIndexPtr()[k] = -1;
            matrix.valuePtr()[k] = 0;
        }
        // Mark the j-th column as empty
        matrix.outerIndexPtr()[j] = -1;
    }
    // Remove the empty columns and update the internal data structure of the matrix
    matrix.prune([](int i, int j, double value) { return value != 0; });
}

void FuncEval(const Eigen::MatrixXd& Pose, const Eigen::MatrixXd& PoseGT, const ParamStruct& ValParam){
    int NumRow = Pose.rows();
    int NumCol = Pose.cols();
    Eigen::MatrixXd PoseDelete(NumRow-1, NumCol);
    Eigen::MatrixXd PoseGTDelete(NumRow-1, NumCol);

    PoseDelete.block(0, 0, NumRow - 1, NumCol) = Pose.block(1, 0, NumRow -1, NumCol);
    PoseGTDelete.block(0, 0, NumRow - 1, NumCol) = PoseGT.block(1, 0, NumRow -1, NumCol);

    Eigen::MatrixXd Trans(NumRow-1, 2), TransGT(NumRow-1, 2);
    Eigen::ArrayXd Rot(NumRow-1), RotGT(NumRow-1);

    Trans.col(0) = PoseDelete.col(0);
    Trans.col(1) = PoseDelete.col(1);
    TransGT.col(0) = PoseGTDelete.col(0);
    TransGT.col(1) = PoseGTDelete.col(1);
    Rot = PoseDelete.col(2).array();
    FuncBatchWrap(Rot);
    RotGT = PoseGTDelete.col(2).array();

    double MAETrans = (TransGT.array() - Trans.array()).abs().mean();
    double MAERot = (RotGT - Rot).abs().mean();

    std::cout<<"MAE of Translation is"<< "  "<<MAETrans<<std::endl;
    std::cout<<"MAE of Rotation is"<< "  "<<MAERot<<std::endl;

    Eigen::Map<Eigen::ArrayXd> ArrTrans(Trans.data(), Trans.size());
    Eigen::Map<Eigen::ArrayXd> ArrTransGT(TransGT.data(), TransGT.size());

    double RMSETrans = sqrt((ArrTransGT - ArrTrans).pow(2).sum() / (ArrTrans.size()));
    double RMSERot = sqrt((RotGT - Rot).pow(2).sum() / (RotGT.size()));

    std::cout<<"RMSE of Translation is"<< "  "<<RMSETrans<<std::endl;
    std::cout<<"RMSE of Rotation is"<< "  "<<RMSERot<<std::endl;
}

void FuncNoisePose(Eigen::MatrixXd& Pose){
    double mean = 0.0;
    double N_T = 2;
    double N_R = 0.3;
    int nP = Pose.rows();
    Pose.block(1, 0, nP - 1, 2) += (Eigen::MatrixXd::Random(nP - 1, 2)) * N_T;
    Pose.block(1, 2, nP - 1, 1) += (Eigen::MatrixXd::Random(nP - 1, 1)) * N_R;
}


void FuncBatchWrap(Eigen::ArrayXd& alpha) {
    int nn = alpha.size();

    for (int i = 0; i < nn; i++)
    {
        if (alpha(i) > M_PI) {
            alpha(i) = alpha(i) - 2 * M_PI;
        }
        else if (alpha(i) < -M_PI)
        {
            alpha(i) = alpha(i) + 2 * M_PI;
        }
    }

}

// No odometry inputs
std::tuple<Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::ArrayXd,double,double> FuncDiffJacobian(const Eigen::MatrixXd& Map, const Eigen::MatrixXd& N, const Eigen::MatrixXd& Pose, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, const ParamStruct& ValParam){
    int NumPose = Pose.size()/3;
    // Calculate the gradient of map
    double h = 1.0;
    Eigen::MatrixXd G = FuncGradient(Map,h);
    Eigen::MatrixXd Gv = G.block(0, 0, Map.rows(), Map.cols());
    Eigen::MatrixXd Gu = G.block(0, Map.cols(), Map.rows(), Map.cols());

    int nPts = 0;

    Eigen::ArrayXi JPID1;
    Eigen::ArrayXi JPID2;
    Eigen::ArrayXd JPVal;

    Eigen::ArrayXi JDID1;
    Eigen::ArrayXi JDID2;
    Eigen::ArrayXd JDVal;

    Eigen::ArrayXi JOID1;
    Eigen::ArrayXi JOID2;
    Eigen::ArrayXd JOVal;

    Eigen::ArrayXd ErrorS;
    Eigen::ArrayXd ErrorO;

    for (int i = 0; i < NumPose; ++i) {
        Eigen::Rotation2Dd rotation(Pose(i,2));
        Eigen::Matrix2d Ri = rotation.toRotationMatrix();
        Eigen::ArrayXd XYi = ScanXY[i];
        Eigen::ArrayXd Oddi = ScanOdd[i];
        Eigen::Map<Eigen::MatrixXd> MatrixXY(XYi.data(), 2, XYi.size()/2);


        Eigen::Vector2d VectorTrans = Pose.block<>(i,0,1,2).transpose();
        Eigen::MatrixXd XY2 = (Ri * MatrixXY).colwise() + VectorTrans;
        Eigen::Vector2d Origin(ValParam.OriginX,ValParam.OriginY);
        Eigen::MatrixXd XY3 = ((XY2.colwise()-Origin).array() / ValParam.Scale).matrix();

        // Interpolation
        Eigen::ArrayXd MapInterpXY3 = FuncBilinearInterpolation(Map, XY3.transpose());
        Eigen::ArrayXd NInterpXY3 = FuncBilinearInterpolation(N, XY3.transpose());

        // Calculation of Error w.r.t. Observations Term
        Eigen::ArrayXd Ei = MapInterpXY3.cwiseQuotient(NInterpXY3) - Oddi;

        ErrorS.conservativeResize(ErrorS.size()+Ei.size());
        ErrorS.tail(Ei.size()) = Ei;


        Eigen::ArrayXd GuInterpXY3 = FuncBilinearInterpolation(Gu, XY3.transpose());
        Eigen::ArrayXd GvInterpXY3 = FuncBilinearInterpolation(Gv, XY3.transpose());

        // Calculation of Jacobian of Observation Terms w.r.t. Poses
        Eigen::MatrixXd dMdXY3(2,NInterpXY3.size());
        dMdXY3.row(0) = GuInterpXY3 / NInterpXY3;
        dMdXY3.row(1) = GvInterpXY3 / NInterpXY3;

        Eigen::Matrix2d dR = FuncdR2D(Pose(i,2));
        Eigen::MatrixXd dXY3dR = dR.transpose() * MatrixXY / ValParam.Scale;
        Eigen::MatrixXd dMdR = (dMdXY3.array() * dXY3dR.array()).matrix().colwise().sum();
        Eigen::MatrixXd dMdT = dMdXY3 / ValParam.Scale;

        Eigen::MatrixXd dMdP(3,dMdR.size());
        dMdP << dMdT, dMdR;

        int nPtsi = Oddi.size();
        Eigen::ArrayXi IDi = Eigen::ArrayXi::LinSpaced(nPtsi, nPts, nPts + nPtsi -1);
        nPts = nPts+nPtsi;
        Eigen::MatrixXi dEdPID1 = IDi.replicate<1,3>();

        Eigen::ArrayXi IDCol = Eigen::ArrayXi::LinSpaced(3, 3*i, 3*(i+1)-1);
        Eigen::MatrixXi dEdPID2 = IDCol.replicate(1, nPtsi).transpose();

        Eigen::ArrayXi JPID1i = Eigen::Map<Eigen::ArrayXi>(dEdPID1.data(), dEdPID1.size());
        Eigen::ArrayXi JPID2i = Eigen::Map<Eigen::ArrayXi>(dEdPID2.data(), dEdPID2.size());
        Eigen::MatrixXd dMdPTranpose = dMdP.transpose();
        Eigen::ArrayXd JPVali = Eigen::Map<Eigen::ArrayXd>(dMdPTranpose.data(), dMdPTranpose.size());

        JPID1.conservativeResize(JPID1.size()+JPID1i.size());
        JPID1.tail(JPID1i.size()) = JPID1i;
        JPID2.conservativeResize(JPID2.size()+JPID2i.size());
        JPID2.tail(JPID2i.size()) = JPID2i;
        JPVal.conservativeResize(JPVal.size()+JPVali.size());
        JPVal.tail(JPVali.size()) = JPVali;


        // Calculation of Jacobian of Observation Terms w.r.t. Map
        Eigen::MatrixXi XY3Floor = XY3.array().floor().cast<int>().matrix();

        Eigen::ArrayXd u = XY3.row(0);
        Eigen::ArrayXd v = XY3.row(1);

        Eigen::ArrayXd u1 = XY3Floor.row(0).cast<double>();
        Eigen::ArrayXd v1 = XY3Floor.row(1).cast<double>();

        Eigen::ArrayXd a = (v1+1-v) * (u1+1-u) / NInterpXY3;
        Eigen::ArrayXd b = (v-v1) * (u1+1-u) / NInterpXY3;
        Eigen::ArrayXd c = (v1+1-v) * (u-u1) / NInterpXY3;
        Eigen::ArrayXd d = (v-v1) * (u-u1) / NInterpXY3;

        Eigen::MatrixXd dEdM(4,u.size());
        dEdM << Eigen::Map<Eigen::ArrayXXd>(a.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXd>(b.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXd>(c.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXd>(d.data(), 1, u.size());

        Eigen::MatrixXd dEdMID2(4,u.size());

        a = ValParam.Sizej * v1 + u1;
        b = ValParam.Sizej * (v1 + 1) + u1;
        c = ValParam.Sizej * v1 + u1 + 1;
        d = ValParam.Sizej * (v1 + 1) + u1 +1;

        dEdMID2 << Eigen::Map<Eigen::ArrayXXd>(a.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXd>(b.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXd>(c.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXd>(d.data(), 1, u.size());

        Eigen::MatrixXi dEdMID1 = IDi.replicate<1,4>();
        Eigen::ArrayXi JDID1i = Eigen::Map<Eigen::ArrayXi>(dEdMID1.data(), dEdMID1.size());

        Eigen::MatrixXi dEdMID2Tranp = dEdMID2.transpose().cast<int>();
        Eigen::ArrayXi JDID2i = Eigen::Map<Eigen::ArrayXi>(dEdMID2Tranp.data(), dEdMID2Tranp.size());

        Eigen::MatrixXd dEdMTranp = dEdM.transpose();
        Eigen::ArrayXd JDVali = Eigen::Map<Eigen::ArrayXd>(dEdMTranp.data(), dEdMTranp.size());

        JDID1.conservativeResize(JDID1.size()+JDID1i.size());
        JDID1.tail(JDID1i.size()) = JDID1i;
        JDID2.conservativeResize(JDID2.size()+JDID2i.size());
        JDID2.tail(JDID2i.size()) = JDID2i;
        JDVal.conservativeResize(JDVal.size()+JDVali.size());
        JDVal.tail(JDVali.size()) = JDVali;
    }

    Eigen::SparseMatrix<double> JP(ErrorS.size(),3*NumPose), JD(ErrorS.size(),ValParam.Sizei*ValParam.Sizej);
    std::thread t1([&]() {
        igl::sparse(JPID1, JPID2, JPVal, ErrorS.size(), 3 * NumPose, JP);
    });
    std::thread t2([&]() {
        igl::sparse(JDID1, JDID2, JDVal, ErrorS.size(), ValParam.Sizei * ValParam.Sizej, JD);
    });
    t1.join();
    t2.join();

    JP.makeCompressed();
    JD.makeCompressed();

    Eigen::SparseMatrix<double> IS = FuncGetI(ErrorS);
    IS.makeCompressed();

    Eigen::MatrixXd MatErrorS = Eigen::MatrixXd::Map(ErrorS.data(), ErrorS.size(), 1);

    double SumErrorS = (MatErrorS.transpose() * IS * MatErrorS)(0,0);
    double SumError = SumErrorS;
    double MeanError = SumErrorS/ErrorS.size();

    return std::make_tuple(JP,JD,IS,ErrorS,SumError,MeanError);
}

// no odometry
Eigen::SparseMatrix<double> FuncGetI(const Eigen::VectorXd& ErrorS)
{
    double Sigma_S = 1.0;
    int nS = ErrorS.size();
    Eigen::ArrayXd diagSigmaS = Eigen::ArrayXd::Constant(nS,Sigma_S);
    Eigen::SparseMatrix<double> IS(nS,nS);
    IS.setIdentity();
    IS.diagonal() = diagSigmaS;
    return IS;
}

// no odom
std::tuple<Eigen::VectorXd, Eigen::VectorXd, double, double, double, double> FuncDelta(const Eigen::MatrixXd& Map, const Eigen::SparseMatrix<double>& JP, const Eigen::SparseMatrix<double>& JD, const Eigen::ArrayXd& ErrorS, const Eigen::SparseMatrix<double>& IS, const Eigen::SparseMatrix<double>& WeightHH, const ParamStruct& ValParam)
{
    Eigen::SparseMatrix<double,Eigen::RowMajor> JPSlice = JP.block(0, 3, JP.rows(), JP.cols()-3);
    Eigen::SparseMatrix<double,Eigen::RowMajor> JDRow(JD);
    JPSlice.makeCompressed();
    JDRow.makeCompressed();
    Eigen::SparseMatrix<double> U = JPSlice.transpose() * JPSlice;
    Eigen::SparseMatrix<double> V = JDRow.transpose() * JDRow + WeightHH;
    Eigen::SparseMatrix<double> W = JPSlice.transpose() * JDRow;

    Eigen::VectorXd VecErrorS = Eigen::VectorXd::Map(ErrorS.data(), ErrorS.size());
    Eigen::VectorXd EP = -JPSlice.transpose() * VecErrorS;
    Eigen::VectorXd ED = -JDRow.transpose() * VecErrorS;
    Eigen::VectorXd XH0 = Map.transpose().reshaped(Map.size(),1);
    Eigen::VectorXd EH = -WeightHH * XH0;
    Eigen::VectorXd EDEH = ED + EH;

    Eigen::SparseMatrix<double> UW = igl::cat(2, U, W);
    Eigen::SparseMatrix<double> WT = W.transpose();
    Eigen::SparseMatrix<double> WV = igl::cat(2, WT, V);
    Eigen::SparseMatrix<double> II = igl::cat(1, UW, WV);
    Eigen::VectorXd EE = igl::cat(1,EP,EDEH);

    Eigen::initParallel();
    II.makeCompressed();
    Eigen::VectorXd Delta;
    if (ValParam.ModeSparseSolver){
        Eigen::ConjugateGradient <Eigen::SparseMatrix<double>, Eigen::Upper|Eigen::Lower> solver;
        int MaxNum = floor(sqrt(II.rows()));
        if (MaxNum > ValParam.SolverFirstMaxIter){MaxNum = ValParam.SolverFirstMaxIter;}
        solver.setMaxIterations(MaxNum);
        solver.setTolerance(ValParam.SolverFirstTolerance);
        solver.compute(II);
        Delta = solver.solve(EE);
    }
    else{
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
        solver.compute(II);
        Delta = solver.solve(EE);
    }

    Eigen::VectorXd DeltaP = Delta.head(JPSlice.cols());
    Eigen::VectorXd DeltaD = Delta.tail(Delta.size() - JPSlice.cols());

    double SumDelta = Delta.transpose() * Delta;
    double MeanDelta = SumDelta / Delta.size();
    double SumDeltaP = DeltaP.transpose() * DeltaP;
    double MeanDeltaP = SumDeltaP / DeltaP.size();

    return std::make_tuple(DeltaP,DeltaD,SumDelta,MeanDelta,SumDeltaP,MeanDeltaP);
}


// no odom
std::tuple<Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::ArrayXd,double,double> FuncDiffSelectJacobian(const Eigen::MatrixXd& SelectMap, const Eigen::MatrixXd& SelectN, const Eigen::MatrixXd& Pose, const std::vector<Eigen::ArrayXd>& SelectScanXY, const std::vector<Eigen::ArrayXd>& SelectScanOdd, const Eigen::MatrixXi& IdSelectVar, const ParamStruct& ValParam){
    int NumPose = Pose.size()/3;
    // Calculate the gradient of map
    double h = 1.0;
    Eigen::MatrixXd G = FuncGradient(SelectMap,h);
    Eigen::MatrixXd Gv = G.block(0, 0, SelectMap.rows(), SelectMap.cols());
    Eigen::MatrixXd Gu = G.block(0, SelectMap.cols(), SelectMap.rows(), SelectMap.cols());

    Eigen::ArrayXi RowIdSelectVar = IdSelectVar.col(0);
    Eigen::ArrayXi ColIdSelectVar = IdSelectVar.col(1);
    Eigen::MatrixXi OneIdSelectVar(RowIdSelectVar.size(),1);
    Eigen::ArrayXi ArraySelectId = RowIdSelectVar * ValParam.Sizej + ColIdSelectVar;
    std::sort(ArraySelectId.data(), ArraySelectId.data() + ArraySelectId.size());
    OneIdSelectVar.col(0) = ArraySelectId;

    int nPts = 0;

    Eigen::ArrayXi JPID1;
    Eigen::ArrayXi JPID2;
    Eigen::ArrayXd JPVal;

    Eigen::ArrayXi JDID1;
    Eigen::ArrayXi JDID2;
    Eigen::ArrayXd JDVal;

    Eigen::ArrayXi JOID1;
    Eigen::ArrayXi JOID2;
    Eigen::ArrayXd JOVal;

    Eigen::ArrayXd ErrorS;
    Eigen::ArrayXd ErrorO;

    for (int i = 0; i < NumPose; ++i) {
        Eigen::Rotation2Dd rotation(Pose(i,2));
        Eigen::Matrix2d Ri = rotation.toRotationMatrix();
        Eigen::ArrayXd XYi = SelectScanXY[i];
        Eigen::ArrayXd Oddi = SelectScanOdd[i];
        Eigen::MatrixXd MatrixXY = Eigen::Map<Eigen::MatrixXd>(XYi.data(), 2, XYi.size()/2);
        Eigen::Vector2d VectorTrans = Pose.block<>(i,0,1,2).transpose();
        Eigen::MatrixXd XY2 = (Ri * MatrixXY).colwise() + VectorTrans;
        Eigen::Vector2d Origin(ValParam.OriginX,ValParam.OriginY);
        Eigen::MatrixXd XY3 = ((XY2.colwise()-Origin).array() / ValParam.Scale).matrix();

        //check if some points out of selected variables
        Eigen::MatrixXi UV =  XY3.array().floor().cast<int>().matrix();

        Eigen::ArrayXd u = XY3.row(0);
        Eigen::ArrayXd v = XY3.row(1);

        Eigen::ArrayXd u1 = UV.row(0).cast<double>();
        Eigen::ArrayXd v1 = UV.row(1).cast<double>();

        Eigen::ArrayXd aa = ValParam.Sizej * v1 + u1;
        Eigen::ArrayXd bb = ValParam.Sizej * (v1 + 1) + u1;
        Eigen::ArrayXd cc = ValParam.Sizej * v1 + u1 + 1;
        Eigen::ArrayXd dd = ValParam.Sizej * (v1 + 1) + u1 +1;

        Eigen::ArrayXi pa, pb, pc, pd;
        Eigen::ArrayXi Boola, Boolb, Boolc, Boold;
        Eigen::MatrixXi Mata(aa.size(),1), Matb(bb.size(),1), Matc(aa.size(),1), Matd(bb.size(),1);
        Mata.col(0) = aa.cast<int>();
        Matb.col(0) = bb.cast<int>();
        Matc.col(0) = cc.cast<int>();
        Matd.col(0) = dd.cast<int>();
        igl::ismember(Mata, OneIdSelectVar, Boola, pa);
        igl::ismember(Matb, OneIdSelectVar, Boolb, pb);
        igl::ismember(Matc, OneIdSelectVar, Boolc, pc);
        igl::ismember(Matd, OneIdSelectVar, Boold, pd);

        Eigen::ArrayXi BoolOut = Boola * Boolb * Boolc * Boold;
        Eigen::ArrayXi OutId;
        igl::find(BoolOut==0, OutId);

        if (OutId.size()!=0){
            RemoveArrayIndex(Oddi, OutId);
            RemoveArrayIndex(pa, OutId);
            RemoveArrayIndex(pb, OutId);
            RemoveColumn(XY3, OutId);
            RemoveColumn(MatrixXY, OutId);
        }
        pc = pa + 1;
        pd = pb + 1;
        // Interpolation
        Eigen::ArrayXd MapInterpXY3 = FuncBilinearInterpolation(SelectMap, XY3.transpose());
        Eigen::ArrayXd NInterpXY3 = FuncBilinearInterpolation(SelectN, XY3.transpose());
        // Calculation of Error w.r.t. Observations Term
        Eigen::ArrayXd Ei = MapInterpXY3.cwiseQuotient(NInterpXY3) - Oddi;

        ErrorS.conservativeResize(ErrorS.size()+Ei.size());
        ErrorS.tail(Ei.size()) = Ei;

        Eigen::ArrayXd GuInterpXY3 = FuncBilinearInterpolation(Gu, XY3.transpose());
        Eigen::ArrayXd GvInterpXY3 = FuncBilinearInterpolation(Gv, XY3.transpose());

        // Calculation of Jacobian of Observation Terms w.r.t. Poses
        Eigen::MatrixXd dMdXY3(2,NInterpXY3.size());
        dMdXY3.row(0) = GuInterpXY3 / NInterpXY3;
        dMdXY3.row(1) = GvInterpXY3 / NInterpXY3;
        Eigen::Matrix2d dR = FuncdR2D(Pose(i,2));
        Eigen::MatrixXd dXY3dR = dR.transpose() * MatrixXY / ValParam.Scale;
        Eigen::MatrixXd dMdR = (dMdXY3.array() * dXY3dR.array()).matrix().colwise().sum();
        Eigen::MatrixXd dMdT = dMdXY3 / ValParam.Scale;
        Eigen::MatrixXd dMdP(3,dMdR.size());
        dMdP << dMdT, dMdR;

        int nPtsi = Oddi.size();
        Eigen::ArrayXi IDi = Eigen::ArrayXi::LinSpaced(nPtsi, nPts, nPts + nPtsi -1);
        nPts = nPts+nPtsi;
        Eigen::MatrixXi dEdPID1 = IDi.replicate<1,3>();

        Eigen::ArrayXi IDCol = Eigen::ArrayXi::LinSpaced(3, 3*i, 3*(i+1)-1);
        Eigen::MatrixXi dEdPID2 = IDCol.replicate(1, nPtsi).transpose();

        Eigen::ArrayXi JPID1i = Eigen::Map<Eigen::ArrayXi>(dEdPID1.data(), dEdPID1.size());
        Eigen::ArrayXi JPID2i = Eigen::Map<Eigen::ArrayXi>(dEdPID2.data(), dEdPID2.size());
        Eigen::MatrixXd dMdPTranpose = dMdP.transpose();
        Eigen::ArrayXd JPVali = Eigen::Map<Eigen::ArrayXd>(dMdPTranpose.data(), dMdPTranpose.size());

        JPID1.conservativeResize(JPID1.size()+JPID1i.size());
        JPID1.tail(JPID1i.size()) = JPID1i;
        JPID2.conservativeResize(JPID2.size()+JPID2i.size());
        JPID2.tail(JPID2i.size()) = JPID2i;
        JPVal.conservativeResize(JPVal.size()+JPVali.size());
        JPVal.tail(JPVali.size()) = JPVali;


        // Calculation of Jacobian of Observation Terms w.r.t. Map
        // Delete UV, u, v, u1, v1;
        UV =  XY3.array().floor().cast<int>().matrix();

        u = XY3.row(0);
        v = XY3.row(1);
        u1 = UV.row(0).cast<double>();
        v1 = UV.row(1).cast<double>();

        Eigen::ArrayXd a = (v1+1-v) * (u1+1-u) / NInterpXY3;
        Eigen::ArrayXd b = (v-v1) * (u1+1-u) / NInterpXY3;
        Eigen::ArrayXd c = (v1+1-v) * (u-u1) / NInterpXY3;
        Eigen::ArrayXd d = (v-v1) * (u-u1) / NInterpXY3;

        Eigen::MatrixXd dEdM(4,u.size());
        dEdM << Eigen::Map<Eigen::ArrayXXd>(a.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXd>(b.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXd>(c.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXd>(d.data(), 1, u.size());

        Eigen::MatrixXi dEdMID2(4,u.size());

        dEdMID2 << Eigen::Map<Eigen::ArrayXXi>(pa.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXi>(pb.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXi>(pc.data(), 1, u.size()),
                Eigen::Map<Eigen::ArrayXXi>(pd.data(), 1, u.size());
        Eigen::MatrixXi dEdMID1 = IDi.replicate<1,4>();
        Eigen::ArrayXi JDID1i = Eigen::Map<Eigen::ArrayXi>(dEdMID1.data(), dEdMID1.size());


        Eigen::MatrixXi dEdMID2Tranp = dEdMID2.transpose().cast<int>();
        Eigen::ArrayXi JDID2i = Eigen::Map<Eigen::ArrayXi>(dEdMID2Tranp.data(), dEdMID2Tranp.size());
        Eigen::MatrixXd dEdMTranp = dEdM.transpose();
        Eigen::ArrayXd JDVali = Eigen::Map<Eigen::ArrayXd>(dEdMTranp.data(), dEdMTranp.size());

        JDID1.conservativeResize(JDID1.size()+JDID1i.size());
        JDID1.tail(JDID1i.size()) = JDID1i;
        JDID2.conservativeResize(JDID2.size()+JDID2i.size());
        JDID2.tail(JDID2i.size()) = JDID2i;
        JDVal.conservativeResize(JDVal.size()+JDVali.size());
        JDVal.tail(JDVali.size()) = JDVali;
    }

    Eigen::SparseMatrix<double> JP(ErrorS.size(),3*NumPose), JD(ErrorS.size(),ValParam.Sizei*ValParam.Sizej);
    std::thread t1([&]() {
        igl::sparse(JPID1, JPID2, JPVal, ErrorS.size(), 3 * NumPose, JP);
    });
    std::thread t2([&]() {
        igl::sparse(JDID1, JDID2, JDVal, ErrorS.size(), ArraySelectId.size(), JD);
    });
    t1.join();
    t2.join();

    JP.makeCompressed();
    JD.makeCompressed();

    Eigen::SparseMatrix<double> IS = FuncGetI(ErrorS);
    IS.makeCompressed();

    Eigen::MatrixXd MatErrorS = Eigen::MatrixXd::Map(ErrorS.data(), ErrorS.size(), 1);

    double SumErrorS = (MatErrorS.transpose() * IS * MatErrorS)(0,0);
    double SumError = SumErrorS;
    double MeanError = SumErrorS/ErrorS.size();
    return std::make_tuple(JP,JD,IS,ErrorS,SumError,MeanError);
}

//no odom

std::tuple<Eigen::VectorXd, Eigen::VectorXd, double, double, double, double> FuncSelectMapDelta(const Eigen::MatrixXd& SelectMap, const Eigen::SparseMatrix<double>& JP, const Eigen::SparseMatrix<double>& JD, const Eigen::ArrayXd& ErrorS, const Eigen::SparseMatrix<double>& IS, const Eigen::SparseMatrix<double>& WeightHH, const Eigen::MatrixXi& IdSelectVar, const ParamStruct& ValParam){
    Eigen::SparseMatrix<double,Eigen::RowMajor> JPSlice = JP.block(0, 3, JP.rows(), JP.cols()-3);
    Eigen::SparseMatrix<double,Eigen::RowMajor> JDRow(JD);
    JPSlice.makeCompressed();
    JDRow.makeCompressed();
    Eigen::SparseMatrix<double> U = JPSlice.transpose() * JPSlice ;
    Eigen::SparseMatrix<double> V = JDRow.transpose() * JDRow + WeightHH;
    Eigen::SparseMatrix<double> W = JPSlice.transpose() * JDRow;

    Eigen::VectorXd VecErrorS = Eigen::VectorXd::Map(ErrorS.data(), ErrorS.size());
    Eigen::VectorXd EP = -JPSlice.transpose()  * VecErrorS;

    Eigen::VectorXd ED = -JDRow.transpose()  * VecErrorS;
    Eigen::ArrayXi RowIdSelectVar = IdSelectVar.col(0);
    Eigen::ArrayXi ColIdSelectVar = IdSelectVar.col(1);
    // Sort the order to variables order
    Eigen::ArrayXi ArraySortSelectId = RowIdSelectVar * ValParam.Sizej + ColIdSelectVar;
    std::sort(ArraySortSelectId.data(), ArraySortSelectId.data() + ArraySortSelectId.size());

    Eigen::ArrayXd SortSelectMap = SelectMap.transpose().reshaped(ValParam.Sizei*ValParam.Sizej,1);
    Eigen::VectorXd XH0 = SortSelectMap(ArraySortSelectId);

    Eigen::VectorXd EH = -WeightHH * XH0;
    Eigen::VectorXd EDEH = ED + EH;

    Eigen::SparseMatrix<double> UW = igl::cat(2, U, W);
    Eigen::SparseMatrix<double> WT = W.transpose();
    Eigen::SparseMatrix<double> WV = igl::cat(2, WT, V);
    Eigen::SparseMatrix<double> II = igl::cat(1, UW, WV);
    Eigen::VectorXd EE = igl::cat(1,EP,EDEH);

    II.makeCompressed();
    Eigen::initParallel();
    Eigen::ConjugateGradient <Eigen::SparseMatrix<double>, Eigen::Upper|Eigen::Lower> solver;
    int MaxNum = floor(sqrt(II.rows()));
    if (MaxNum > ValParam.SolverSecondMaxIter){MaxNum = ValParam.SolverSecondMaxIter;}
    solver.setMaxIterations(MaxNum);
    solver.setTolerance(ValParam.SolverSecondTolerance);
    solver.compute(II);
    Eigen::VectorXd Delta = solver.solve(EE);

    Eigen::VectorXd DeltaP = Delta.head(JPSlice.cols());
    Eigen::VectorXd DeltaD = Delta.tail(Delta.size() - JPSlice.cols());

    double SumDelta = Delta.transpose() * Delta;
    double MeanDelta = SumDelta / Delta.size();
    double SumDeltaP = DeltaP.transpose() * DeltaP;
    double MeanDeltaP = SumDeltaP / DeltaP.size();

    return std::make_tuple(DeltaP,DeltaD,SumDelta,MeanDelta,SumDeltaP,MeanDeltaP);
}


void FuncKeyFrameSelectionOdom(const Eigen::MatrixXd& Pose, Eigen::MatrixXd& PoseTem, const Eigen::MatrixXd& PoseGT, Eigen::MatrixXd& PoseGTTem, const Eigen::MatrixXd& Odom, Eigen::MatrixXd& OdomTem, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, std::vector<Eigen::ArrayXd>& ScanXYTem, std::vector<Eigen::ArrayXd>& ScanOddTem, const ParamStruct& ValParam){
    if (ValParam.KeyframeRate > 1){
        int NumPose = Pose.rows();
        int NumSelect = floor((NumPose/ValParam.KeyframeRate)) + 1;
        
        // Select Pose and Scan

        int kk = 0;
        for (int i = 0; i < NumPose; i+=ValParam.KeyframeRate) {
            PoseTem.row(kk) = Pose.row(i);
            PoseGTTem.row(kk) = PoseGT.row(i);
            ScanXYTem[kk] = ScanXY[i];
            ScanOddTem[kk] = ScanOdd[i];
            kk++;
        }
        // Convert Odom from Increments to Poses
        Eigen::MatrixXd OdomPosition(NumPose,3);
        OdomPosition.row(0) = Odom.row(0);
        for (int i = 1; i < NumPose; ++i) {
            OdomPosition.row(i).segment(0, 2) = (FuncdR2D(OdomPosition(i-1, 2)) * Odom.row(i).segment(0, 2).transpose()).transpose() + OdomPosition.row(i-1).segment(0, 2);
            OdomPosition(i,2) = FuncWrap(OdomPosition(i-1,2) + Odom(i,2));
        }
        // Select Odom
        kk = 0;
        for (int i = 0; i < NumPose; i+=ValParam.KeyframeRate) {
            if (i==0){
                OdomTem.row(0) = Odom.row(0);
            }
            else{
                OdomTem.row(kk).segment(0, 2) = (FuncTheta2R(OdomPosition(i-ValParam.KeyframeRate,2)).transpose() * (OdomPosition.row(i).segment(0,2).transpose() - OdomPosition.row(i-ValParam.KeyframeRate).segment(0,2).transpose())).transpose();
                OdomTem(kk,2) = FuncWrap(OdomPosition(i,2) - OdomPosition(i-ValParam.KeyframeRate,2));
            }
            kk++;
        }
    }
}



void FuncKeyFrameSelectionOdom(const Eigen::MatrixXd& Pose, Eigen::MatrixXd& PoseTem, const Eigen::MatrixXd& Odom, Eigen::MatrixXd& OdomTem, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, std::vector<Eigen::ArrayXd>& ScanXYTem, std::vector<Eigen::ArrayXd>& ScanOddTem, const ParamStruct& ValParam){
    if (ValParam.KeyframeRate > 1){
        int NumPose = Pose.rows();
        int NumSelect = floor((NumPose/ValParam.KeyframeRate)) + 1;

        // Select Pose and Scan

        int kk = 0;
        for (int i = 0; i < NumPose; i+=ValParam.KeyframeRate) {
            PoseTem.row(kk) = Pose.row(i);
            ScanXYTem[kk] = ScanXY[i];
            ScanOddTem[kk] = ScanOdd[i];
            kk++;
        }
        // Convert Odom from Increments to Poses
        Eigen::MatrixXd OdomPosition(NumPose,3);
        OdomPosition.row(0) = Odom.row(0);
        for (int i = 1; i < NumPose; ++i) {
            OdomPosition.row(i).segment(0, 2) = (FuncdR2D(OdomPosition(i-1, 2)) * Odom.row(i).segment(0, 2).transpose()).transpose() + OdomPosition.row(i-1).segment(0, 2);
            OdomPosition(i,2) = FuncWrap(OdomPosition(i-1,2) + Odom(i,2));
        }
        // Select Odom
        kk = 0;
        for (int i = 0; i < NumPose; i+=ValParam.KeyframeRate) {
            if (i==0){
                OdomTem.row(0) = Odom.row(0);
            }
            else{
                OdomTem.row(kk).segment(0, 2) = (FuncTheta2R(OdomPosition(i-ValParam.KeyframeRate,2)).transpose() * (OdomPosition.row(i).segment(0,2).transpose() - OdomPosition.row(i-ValParam.KeyframeRate).segment(0,2).transpose())).transpose();
                OdomTem(kk,2) = FuncWrap(OdomPosition(i,2) - OdomPosition(i-ValParam.KeyframeRate,2));
            }
            kk++;
        }
    }
}



void FuncKeyFrameSelection(const Eigen::MatrixXd& Pose, Eigen::MatrixXd& PoseTem, const Eigen::MatrixXd& PoseGT, Eigen::MatrixXd& PoseGTTem, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, std::vector<Eigen::ArrayXd>& ScanXYTem, std::vector<Eigen::ArrayXd>& ScanOddTem, const ParamStruct& ValParam){
    if (ValParam.KeyframeRate > 1){
        int NumPose = Pose.rows();
        int NumSelect = floor((NumPose/ValParam.KeyframeRate)) + 1;

        // Select Pose and Scan

        int kk = 0;
        for (int i = 0; i < NumPose; i+=ValParam.KeyframeRate) {
            PoseTem.row(kk) = Pose.row(i);
            PoseGTTem.row(kk) = PoseGT.row(i);
            ScanXYTem[kk] = ScanXY[i];
            ScanOddTem[kk] = ScanOdd[i];
            kk++;
        }
    }
}



void FuncKeyFrameSelection(const Eigen::MatrixXd& Pose, Eigen::MatrixXd& PoseTem, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, std::vector<Eigen::ArrayXd>& ScanXYTem, std::vector<Eigen::ArrayXd>& ScanOddTem, const ParamStruct& ValParam){
    if (ValParam.KeyframeRate > 1){
        int NumPose = Pose.rows();
        int NumSelect = floor((NumPose/ValParam.KeyframeRate)) + 1;

        // Select Pose and Scan

        int kk = 0;
        for (int i = 0; i < NumPose; i+=ValParam.KeyframeRate) {
            PoseTem.row(kk) = Pose.row(i);
            ScanXYTem[kk] = ScanXY[i];
            ScanOddTem[kk] = ScanOdd[i];
            kk++;
        }
    }
}


void FuncShowMap(Eigen::MatrixXd& Map){
    Eigen::MatrixXd ab = Map.array().exp().matrix();
    Eigen::MatrixXd ExpMap = (1 - ab.array() / (ab.array() + 1)).matrix();
    cv::Mat img = cv::Mat(ExpMap.cols(), ExpMap.rows(), CV_64F, ExpMap.data());
    cv::imshow("Map", img);
    cv::waitKey(1000);
    cv::destroyWindow("Map");
    cv::waitKey(1);
}


void FuncShowMapPress(Eigen::MatrixXd& Map){
    Eigen::MatrixXd ab = Map.array().exp().matrix();
    Eigen::MatrixXd ExpMap = (1 - ab.array() / (ab.array() + 1)).matrix();
    cv::Mat img = cv::Mat(ExpMap.cols(), ExpMap.rows(), CV_64F, ExpMap.data());
    cv::imshow("Map", img);
    cv::waitKey(0);
    cv::destroyWindow("Map");
    cv::waitKey(1);
}


Eigen::MatrixXd FuncInitialiseGridMapToShow(const Eigen::MatrixXd& Pose, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, const ParamStruct& ValParam){
    Eigen::MatrixXd Map = Eigen::MatrixXd::Zero(ValParam.Sizei, ValParam.Sizej);
    int NumPose = Pose.size()/3;
    #pragma omp parallel for
    for (int i = 0; i < NumPose; ++i) {
        Eigen::Rotation2Dd rotation(Pose(i,2));
        Eigen::Matrix2d Ri = rotation.toRotationMatrix();
        Eigen::ArrayXd XYi = ScanXY[i];
        Eigen::ArrayXd Oddi = ScanOdd[i];
        Eigen::Map<Eigen::MatrixXd> MatrixXY(XYi.data(), 2, XYi.size()/2);
        Eigen::Vector2d VectorTrans = Pose.block<>(i,0,1,2).transpose();
        Eigen::MatrixXd Si = (Ri * MatrixXY).colwise() + VectorTrans;
        Eigen::Vector2d Origin(ValParam.OriginX,ValParam.OriginY);
        Eigen::MatrixXd XY3 = ((Si.colwise()-Origin).array() / ValParam.Scale).floor().matrix();

        bool hasNegative = XY3.minCoeff() < 0;
        if (hasNegative){
                std::cout<<"Incorrect Origin Setting"<<std::endl;
        }
        bool hasOverSize = XY3.row(0).maxCoeff() >= ValParam.Sizei || XY3.row(1).maxCoeff() >= ValParam.Sizej;
        if (hasOverSize){
                std::cout<<"Incorrect Map Size Setting"<<std::endl;
        }

        Eigen::MatrixXd TemGlobal = Eigen::MatrixXd::Zero(ValParam.Sizei, ValParam.Sizej);
        Eigen::MatrixXd TemMapN = Eigen::MatrixXd::Zero(ValParam.Sizei, ValParam.Sizej);
        for (int j = 0; j < Oddi.size(); ++j) {
            int tem_col = XY3(0,j);
            int tem_row = XY3(1,j);
            double tem_val = Oddi[j];
            TemGlobal(tem_row, tem_col) = TemGlobal(tem_row, tem_col) + tem_val;
        }
        Map = Map + TemGlobal;
    }
    return Map;
}
void FuncPosefromOdom(const Eigen::MatrixXd& Odom, Eigen::MatrixXd& Pose){
    Pose = Eigen::MatrixXd::Zero(Odom.rows(), 3);
    for (int i = 0; i < Odom.rows(); ++i) {
        if (i==0){
            Pose.row(i) = Odom.row(i);
        }else{
            Pose.row(i).segment(0,2) = (FuncTheta2R(Pose(i-1,2)) * Odom.row(i-1).segment(0,2).transpose()).transpose() + Pose.row(i-1).segment(0,2);
            Pose(i,2) = FuncWrap(Pose(i-1,2) + Odom(i-1,2));
        }
    }
}