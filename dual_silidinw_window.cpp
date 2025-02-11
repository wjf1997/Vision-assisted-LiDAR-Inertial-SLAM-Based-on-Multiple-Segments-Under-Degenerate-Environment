void Estimator::slideMapWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0].stamp.toSec();
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Rs[i].swap(Rs[i + 1]);

                std::swap(pre_integrations[i], pre_integrations[i + 1]);

                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                it_0->second.pre_integration = nullptr;
 
                for (map<double, ImageFrame>::iterator it = all_image_frame.begin(); it != it_0; ++it)
                {
                    if (it->second.pre_integration)
                        delete it->second.pre_integration;
                    it->second.pre_integration = NULL;
                }

                all_image_frame.erase(all_image_frame.begin(), it_0);
                all_image_frame.erase(t_0);

            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
            {
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }

            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            slideWindowNew();
        }
    }
}

void LiDAREstimator::slideEstimationWindow()
{
	if (is_degeneration == false) break;
	else
	{
		Estimation_WINDOW_SIZE = WINDOW_SIZE - OVERLAP_SIZE;
		TicToc t_margin;
		if (degeneration_flag == QUEUE)
		{
			double t_0 = Headers[0].stamp.toSec();
			back_R0 = Rs[0];
			back_P0 = Ps[0];
			if (frame_count == Estimation_WINDOW_SIZE)
			{
				for (int i = 0; i < Estimation_WINDOW_SIZE; i++)
				{
					Rs[i].swap(Rs[i + 1]);

					std::swap(pre_integrations[i], pre_integrations[i + 1]);

					dt_buf[i].swap(dt_buf[i + 1]);
					linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
					angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

					Headers[i] = Headers[i + 1];
					Ps[i].swap(Ps[i + 1]);
					Vs[i].swap(Vs[i + 1]);
					Bas[i].swap(Bas[i + 1]);
					Bgs[i].swap(Bgs[i + 1]);
				}
				Headers[Estimation_WINDOW_SIZE] = Headers[Estimation_WINDOW_SIZE];
				Ps[Estimation_WINDOW_SIZE] = Ps[Estimation_WINDOW_SIZE];
				Vs[Estimation_WINDOW_SIZE] = Vs[Estimation_WINDOW_SIZE - 1];
				Rs[Estimation_WINDOW_SIZE] = Rs[Estimation_WINDOW_SIZE - 1];
				Bas[Estimation_WINDOW_SIZE] = Bas[Estimation_WINDOW_SIZE - 1];
				Bgs[Estimation_WINDOW_SIZE] = Bgs[Estimation_WINDOW_SIZE - 1];

				delete pre_integrations[Estimation_WINDOW_SIZE];
				pre_integrations[Estimation_WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[Estimation_WINDOW_SIZE], Bgs[Estimation_WINDOW_SIZE]};

				dt_buf[Estimation_WINDOW_SIZE].clear();
				linear_acceleration_buf[Estimation_WINDOW_SIZE].clear();
				angular_velocity_buf[Estimation_WINDOW_SIZE].clear();

				if (true || solver_flag == INITIAL)
				{
					map<double, ImageFrame>::iterator it_0;
					it_0 = all_image_frame.find(t_0);
					delete it_0->second.pre_integration;
					it_0->second.pre_integration = nullptr;
	 
					for (map<double, ImageFrame>::iterator it = all_image_frame.begin(); it != it_0; ++it)
					{
						if (it->second.pre_integration)
							delete it->second.pre_integration;
						it->second.pre_integration = NULL;
					}

					all_image_frame.erase(all_image_frame.begin(), it_0);
					all_image_frame.erase(t_0);

				}
				slideWindowOld();
			}
		}
		else if (degeneration_flag == DEQUEUE)
		{
			if (frame_count == Estimation_WINDOW_SIZE)
			{
				for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
				{
					double tmp_dt = dt_buf[frame_count][i];
					Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
					Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

					pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

					dt_buf[frame_count - 1].push_back(tmp_dt);
					linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
					angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
				}

				Headers[frame_count - 1] = Headers[frame_count];
				Ps[frame_count - 1] = Ps[frame_count];
				Vs[frame_count - 1] = Vs[frame_count];
				Rs[frame_count - 1] = Rs[frame_count];
				Bas[frame_count - 1] = Bas[frame_count];
				Bgs[frame_count - 1] = Bgs[frame_count];

				delete pre_integrations[Estimation_WINDOW_SIZE];
				pre_integrations[Estimation_WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[Estimation_WINDOW_SIZE], Bgs[Estimation_WINDOW_SIZE]};

				dt_buf[Estimation_WINDOW_SIZE].clear();
				linear_acceleration_buf[Estimation_WINDOW_SIZE].clear();
				angular_velocity_buf[Estimation_WINDOW_SIZE].clear();

				slideWindowNew();
			}
		}
	}
}


// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}
// real marginalization is removed in solve_ceres()
void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}