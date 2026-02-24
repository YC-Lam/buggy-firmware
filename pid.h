// pid control

class PidControl{
    private:
        // Errors
        float p_error;
        float i_error;
        float d_error;

        // Coefficients
        float Kp;
        float Ki;
        float Kd;

        float integrator;
        float previous_error;
        float integrator_limit;
        float frequency;
    public:
        PidControl(float Kp, float Ki, float Kd);
        ~PidControl();
        float update(float error);
        void reset();
        void set_kp(float kp);
        void set_ki(float ki);
        void set_kd(float kd);
};