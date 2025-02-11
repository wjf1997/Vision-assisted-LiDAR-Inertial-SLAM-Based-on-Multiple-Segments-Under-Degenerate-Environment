void LiDAREstimation::Iteration (const Eigen::Matrix3d &H)
{
	EigenSolver<Matrix3d> solver(H);
    Vector3cd eigenvalues = solver.eigenvalues().real(); 

    for (int i = 0; i < 3; i++) {
        cout << eigenvalues(i) << endl;
    }

    sort(eigenvalues.data(), eigenvalues.data() + 3);

    for (int i = 0; i < 3; i++) {
        cout << eigenvalues(i) << endl;
    }
	
	if (eigenvalues(0)>0.12 && eigenvalues(1)>0.27 && eigenvalues(2)>0.48)
		iteration_degeneration = true;
	else
		iteration_degeneration = false;
}

void LiDAREstimation::JudgeIteration (const &<bool> arr, const %frame)
{
	bool allTrue = true;
    for (int i = 0; i < n; i++) {
        if (!arr[i]) {
            allTrue = false;
            break;
        }
    }
	if (allTrue) 
	{
		quque[frame] = 1;
		return;
	}
        
	bool allFalse = true;
	for (int i = 0; i < n; i++) {
        if (arr[i]) {
            allFalse = false;
            break;
        }
    }
    if (allFalse) {
		quque[frame] = 2;
	return;}
	
	bool fromTrueToFalse = false;
    for (int i = 0; i < n; i++) {
        if (arr[i] == false) {
            fromTrueToFalse = true; 
            break;
        }
    }
	if (fromTrueToFalse)
	{
		quque[frame] = 3;
		return;
	}
		
    bool fromFalseToTrue = false;
    for (int i = 0; i < n; i++) {
        if (arr[i] == true) {
            fromFalseToTrue = true; 
            break;
        }
    }
	if (fromFalseToTrue)
	{
		quque[frame] = 4;
		return;
	}
	return 0;
}


void LiDAREstimation::JudgeQueue (const &<int> queue)
{
	bool found1 = false, found3 = false, found2 = false;
    bool status1 = false; 


    for (int i = 0; i < n; i++) {
        if (!found1 && queue[i] == 1) {
            found1 = true;
        } 
        else if (found1 && !found3 && queue[i] == 3) {
            found3 = true;
        } 
        else if (found3 && !found2 && queue[i] == 2) {
            found2 = true;
        }
        
        if (found1 && found3 && found2) {
            status1 = true;
            break;
        }
    }
	if (status1)
	{
		state = queue_degenerate;
		return;
	}
	
	bool found2 = false, found4 = false, found1 = false;
    bool status1 = false; 


    for (int i = 0; i < n; i++) {
        if (!found2 && queue[i] == 2) {
            found2 = true;
        } 
        else if (found2 && !found4 && queue[i] == 4) {
            found4 = true;
        } 
        else if (found4 && !found1 && queue[i] == 1) {
            found1 = true;
        }
        
        if (found2 && found4 && found1) {
            status1 = true;
            break;
        }
    }

    if (status1)
	{
		state = dequeue_degenerate;
		return;
	}
	return 0;
}