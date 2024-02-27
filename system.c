//	EmSyPro9000

double converter (double u_in){
	static double i1_k = 0;
	static double u1_k = 0;
	static double i2_k = 0;
	static double u2_k = 0;
	static double i3_k = 0;
	static double u3_k = 0;

	// If input parameter equals -1 reset parameters (for when we exit modulation mode)
	if (u_in == -1){
		i1_k = 0;
		u1_k = 0;
		i2_k = 0;
		u2_k = 0;
		i3_k = 0;
		u3_k = 0;
		return -1;
	}

	double i1_kph = 0.9652*i1_k + (-0.0172)*u1_k + 0.0057*i2_k + (-0.0058)*u2_k + 0.0052*i3_k + (-0.0251)*u3_k + 0.0471*u_in;
	double u1_kph = 0.7732*i1_k + 0.1252*u1_k + 0.2315*i2_k + 0.07*u2_k + 0.1282*i3_k + 0.7754*u3_k + 0.0377*u_in;
	double i2_kph = 0.8278*i1_k + (-0.7522)*u1_k + (-0.0956)*i2_k + 0.3299*u2_k + (-0.4855)*i3_k + 0.3915*u3_k + 0.0404*u_in;
	double u2_kph = 0.9948*i1_k + 0.2655*u1_k + (-0.3848)*i2_k + 0.4212*u2_k + 0.3927*i3_k + 0.2899*u3_k + 0.0485*u_in;
	double i3_kph = 0.7648*i1_k + (-0.4165)*u1_k + (-0.4855)*i2_k + (-0.3366)*u2_k + (-0.0986)*i3_k + 0.7281*u3_k + 0.0373*u_in;
	double u3_kph = 1.1056*i1_k + 0.7587*u1_k + 0.1179*i2_k + 0.0748*u2_k + (-0.2192)*i3_k + 0.1491*u3_k + 0.0539*u_in;

	i1_k = i1_kph;
	u1_k = u1_kph;
	i2_k = i2_kph;
	u2_k = u2_kph;
	i3_k = i3_kph;
	u3_k = u3_kph;

	return u3_k;
}
