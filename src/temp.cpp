
//void mapping::matrixMul(double *m1, double *m2, double *m_erg) {
//	//math::Vec<9, double> matrix(m1[0],
//	math::Mat<3,3,double> test_1(m1[0], m1[3],m1[6], m1[1], m1[4],m1[7],m1[2],m1[5],m1[8]);
//	math::Mat<3,3,double> test_2(m2[0], m2[3],m2[6], m2[1], m2[4],m2[7],m2[2],m2[5],m2[8]);
//	math::Mat<3,3,double> test_3;
//	test_3 = test_1*test_2;
//	/*cout << "Matrix 1" << endl;
//	for(int i = 0; i < 3; i++) {
//	for(int j = 0; j < 3; j++) {
//	cout << test_1[i][j] << " ";
//	}
//	cout << endl;
//	}
//	cout << "Matrix 2" << endl;
//	for(int i = 0; i < 3; i++) {
//	for(int j = 0; j < 3; j++) {
//	cout << test_2[i][j] << " ";
//	}
//	cout << endl;
//	}*/
//	//cout << "Ergebnis" << endl;
//	for(int i = 0; i < 3; i++) {
//		for(int j = 0; j < 3; j++) {
//			m_erg[i*3+j] = test_3[i][j];
//			//cout << m_erg[i*3+j] << " ";
//		}
//		//cout << endl;
//	}
//}
//void mapping::matrixAdd(double *m1, double *m2, double *m_erg) {
//	math::Mat<3,3,double> test_1(m1[0], m1[3],m1[6], m1[1], m1[4],m1[7],m1[2],m1[5],m1[8]);
//	math::Mat<3,3,double> test_2(m2[0], m2[3],m2[6], m2[1], m2[4],m2[7],m2[2],m2[5],m2[8]);
//	math::Mat<3,3,double> test_3;
//	test_3 = test_1+test_2;
//
//	for(int i = 0; i < 3; i++) {
//		for(int j = 0; j < 3; j++) {
//			m_erg[i*3+j] = test_3[i][j];
//		}
//	}
//}
/*
//Unütz weil sofort GroupSide aufgerufen werden kann??
void mapping::calcInitialVectors(std::deque<measurement> *v) {
	//cout << "berechne initiale vektoren" << endl;
	ArTransform t;
	double tempVec_1[3];
	double tempVec_2[3];
	deque<measurement> tmp;
	for(int i = 0; i < (*v).size()-1; i++) {
		//		cout << "berechne Vektoren zwischen " << i << " und " << i+1 << endl;
		//t.setTransform((*v)[i].roboPose);
		(*v)[i].roboPose.getPose(&tempVec_1[0], &tempVec_1[1], &tempVec_1[2]);
		(*v)[i+1].roboPose.getPose(&tempVec_2[0], &tempVec_2[1], &tempVec_2[2]);
		struct measurement neu;
		//neu.roboPose.setPose(t.doInvTransform((*v)[i].roboPose));
		//t.setTransform((*v)[i+1].roboPose);
		//neu.roboPose.setPose(t.doTransform(neu.roboPose);
		
		//cout << "kopieren der Messungen" << endl;
		//Messungen von i++ fehlen
		for(int n = 0; n < (*v)[i].SonarMeasures.size(); n++) {
			struct measure neu2;
			for(int h = 0; h < 3; h++) {
				neu2.range[h] = (*v)[i].SonarMeasures[n].range[h];
				neu2.SonarPoseGlobal[h] = (*v)[i].SonarMeasures[n].SonarPoseGlobal[h];
				neu2.SonarPoseLocal[h] = (*v)[i].SonarMeasures[n].SonarPoseLocal[h];
			}
			//cout << "kopieren der kovarianz matrix für messungen" << endl;
			for(int p = 0; p < 3; p++) {
				for(int o = 0; o<3;o++) {
					neu2.measure_cov[p*3+o] = (*v)[i].SonarMeasures[n].measure_cov[p*3+o];
				}
			}
			neu.SonarMeasures.push_back(neu2);
		}
		for(int n = 0; n < (*v)[i+1].SonarMeasures.size(); n++) {
			struct measure neu2;
			for(int h = 0; h < 3; h++) {
				neu2.range[h] = (*v)[i+1].SonarMeasures[n].range[h];
				neu2.SonarPoseGlobal[h] = (*v)[i+1].SonarMeasures[n].SonarPoseGlobal[h];
				neu2.SonarPoseLocal[h] = (*v)[i+1].SonarMeasures[n].SonarPoseLocal[h];
			}
			//cout << "kopieren der kovarianz matrix für messungen" << endl;
			for(int p = 0; p < 3; p++) {
				for(int o = 0; o<3;o++) {
					neu2.measure_cov[p*3+o] = (*v)[i+1].SonarMeasures[n].measure_cov[p*3+o];
				}
			}
			neu.SonarMeasures.push_back(neu2);
		}
		//cout << "vorläufiges setzen der robo kovarianz matrix auf Einheitsmatrix" << endl;
		for(int k = 0; k < 3; k++) {
			for(int j = 0; j < 3; j++) {
				if(k == j) {
					neu.robo_cov[k*3+j] = 1;			
				} else
					neu.robo_cov[k*3+j] = 0;
			}
		}

		//cout << "berechne vektoren zwischen Posen" << endl;
		double tempWinkel = tempVec_2[2] - tempVec_1[2];
		neu.robo_vec[0] = tempVec_2[0] - tempVec_1[0];
		neu.robo_vec[1] = tempVec_2[1] - tempVec_1[1];
		//neu.distance = sqrt((neu.robo_vec[0]*neu.robo_vec[0])+(neu.robo_vec[1]*neu.robo_vec[1]));
		neu.robo_vec[0] = neu.roboPose.getX();
		neu.robo_vec[1] = neu.roboPose.getY();
		neu.robo_vec[2] = neu.roboPose.getThRad();
		if(tempVec_2[2] > tempVec_1[2]) {
			neu.robo_vec[2] = tempVec_1[2] + tempWinkel;
			neu.roboPose.setPose(neu.robo_vec[0], neu.robo_vec[1], neu.robo_vec[2]);
		} else {
			neu.robo_vec[2] = tempVec_1[2] - tempWinkel;
			neu.roboPose.setPose(neu.robo_vec[0], neu.robo_vec[1], neu.robo_vec[2]);
		}
		cout << "Veraenderung der Messpunkte aus " << endl;
		cout << (*v)[i].roboPose.getX() << " "<< (*v)[i].roboPose.getY()<< " " << (*v)[i].roboPose.getTh() << " und" << endl;
		cout << (*v)[i+1].roboPose.getX() << " "<< (*v)[i+1].roboPose.getY()<<" "<< (*v)[i+1].roboPose.getTh()<< " wurde " << endl;
		cout << neu.roboPose.getX() << " " << neu.roboPose.getY() << " " << neu.roboPose.getTh() << endl;
		_getch();
		tmp.push_back(neu);
	}
	tmp.swap((*v));
}
*/