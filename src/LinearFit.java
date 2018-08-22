import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.SwingUtilities;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.random.JDKRandomGenerator;
import org.apache.commons.math3.random.RandomGenerator;
import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;


@SuppressWarnings("serial")
public class LinearFit extends JPanel{
	class Point3D{
		public double x,y,t;
	}
	List<Point3D> points = new ArrayList<Point3D>();
	List<Point3D> ppoints = new ArrayList<Point3D>();
	protected static boolean render = true;
	JLabel minxl= new JLabel("Min x");;
	JTextField minx= new JTextField(20);;
	JLabel maxxl = new JLabel("Max x");;
	JTextField maxx =new JTextField("3",20);
	JButton doRender= new JButton("Render");
	JPanel canvas = new JPanel();
	JFrame jfrm = new JFrame("Diamond");
	static LinearFit diamond;
	LinearFit(){	
		setBorder(BorderFactory.createLineBorder(Color.BLACK, 4));
	}

	@Override
	protected void paintComponent(Graphics g) {
		super.paintComponent(g);
		int height = getHeight();
		int width = getWidth();
		g.setColor(Color.RED);
		double max_x = Double.parseDouble(maxx.getText());
		for (Point3D point: points) {
			g.drawLine((int)Math.round(point.x*max_x-max_x/2.0)+width/2, (int)Math.round(point.y*max_x)+height/2, (int)Math.round(point.x*max_x+max_x/2.0)+width/2,(int)Math.round(point.y*max_x)+height/2);
			g.drawLine((int)Math.round(point.x*max_x)+width/2, (int)Math.round(point.y*max_x-max_x/2.0)+height/2, (int)Math.round(point.x*max_x)+width/2,(int)Math.round(point.y*max_x+max_x/2.0)+height/2);
		}
		g.setColor(Color.BLUE);
		for (Point3D ppoint: ppoints) {
			g.drawLine((int)Math.round(ppoint.x*max_x-max_x/2.0)+width/2, (int)Math.round(ppoint.y*max_x)+height/2, (int)Math.round(ppoint.x*max_x+max_x/2.0)+width/2,(int)Math.round(ppoint.y*max_x)+height/2);
			g.drawLine((int)Math.round(ppoint.x*max_x)+width/2, (int)Math.round(ppoint.y*max_x-max_x/2.0)+height/2, (int)Math.round(ppoint.x*max_x)+width/2,(int)Math.round(ppoint.y*max_x+max_x/2.0)+height/2);
		}


	}
	private static final double NOISE_VALUE = 3.0;
	void doASquare() throws InterruptedException {
		double x,y;
		double t;

		x = 0.0; y= 0.0;
		t = 0.0;
		for (int i=0; i < 20;i++) {
			x -= 2.0;
			t += 1.0;
			predictAndDraw(addNoise(x),addNoise(y),t);

		}		
		for (int i=0; i < 20;i++) {
			y -= 2.0;
			t += 1.0;
			predictAndDraw(addNoise(x),addNoise(y),t);

		}		
		for (int i=0; i < 20;i++) {
			x += 2.0;
			t += 1.0;
			predictAndDraw(addNoise(x),addNoise(y),t);

		}
		for (int i=0; i < 20;i++) {
			y += 2.0;
			t += 1.0;
			predictAndDraw(addNoise(x),addNoise(y),t);

		}		

	}
	//===========================================================================================================================
	/**
	 * A Kalman filter implemented using SimpleMatrix.  The code tends to be easier to
	 * read and write, but the performance is degraded due to excessive creation/destruction of
	 * memory and the use of more generic algorithms.  This also demonstrates how code can be
	 * seamlessly implemented using both SimpleMatrix and DMatrixRMaj.  This allows code
	 * to be quickly prototyped or to be written either by novices or experts.
	 *
	 * @author Peter Abeles
	 */
	public class KalmanFilterSimple{

	    // kinematics description
	    private SimpleMatrix F,Q,H;

	    // sytem state estimate
	    private SimpleMatrix x,P;

	    public void configure(DMatrixRMaj F, DMatrixRMaj Q, DMatrixRMaj H) {
	        this.F = new SimpleMatrix(F);
	        this.Q = new SimpleMatrix(Q);
	        this.H = new SimpleMatrix(H);
	    }

	    public void setState(DMatrixRMaj x, DMatrixRMaj P) {
	        this.x = new SimpleMatrix(x);
	        this.P = new SimpleMatrix(P);
	    }

	    public void predict() {
	        // x = F x
	        x = F.mult(x);

	        // P = F P F' + Q
	        P = F.mult(P).mult(F.transpose()).plus(Q);
	    }

	    public void update(DMatrixRMaj _z, DMatrixRMaj _R) {
	        // a fast way to make the matrices usable by SimpleMatrix
	        SimpleMatrix z = SimpleMatrix.wrap(_z);
	        SimpleMatrix R = SimpleMatrix.wrap(_R);

	        // y = z - H x
	        SimpleMatrix y = z.minus(H.mult(x));

	        // S = H P H' + R
	        SimpleMatrix S = H.mult(P).mult(H.transpose()).plus(R);

	        // K = PH'S^(-1)
	        SimpleMatrix K = P.mult(H.transpose().mult(S.invert()));

	        // x = x + Ky
	        x = x.plus(K.mult(y));

	        // P = (I-kH)P = P - KHP
	        P = P.minus(K.mult(H).mult(P));
	    }

	    public DMatrixRMaj getState() {
	        return x.getMatrix();
	    }

	    public DMatrixRMaj getCovariance() {
	        return P.getMatrix();
	    }
	}
	// discrete time interval
	
	double dt = 1d;
	// position measurement noise (meter)
	double measurementNoise = 1.0d;
	// acceleration noise (meter/sec^2)
	double accelNoise = 0.0d;
	private KalmanFilterSimple filterX;
	private KalmanFilterSimple filterY;
	RealVector u;
	DMatrixRMaj x1;
	DMatrixRMaj y1;
	DMatrixRMaj Ax;
	RealMatrix B;
	DMatrixRMaj Hx;
	DMatrixRMaj Ry;
	DMatrixRMaj Rx;
	private DMatrixRMaj P0y;
	private DMatrixRMaj P0x;
	private DMatrixRMaj Ay;
	private DMatrixRMaj Hy;
	DMatrixRMaj Qy;
	DMatrixRMaj Qx;
	private double vx;
	private double vy;
	class KalmanX {

		public KalmanX(double x, double vx) {
			// A = [ 1 dt ]
			//	     [ 0  1 ]
			Ax = new DMatrixRMaj(new double[][] { { 1, dt }, { 0, 1 } });
			// B = [ dt^2/2 ]
			//	     [ dt     ]
			B = new Array2DRowRealMatrix(new double[][] { { Math.pow(dt, 2d) / 2d }, { dt } });
			// H = [ 1 0 ]
			Hx = new DMatrixRMaj(new double[][] { { 1d, 0d } });
			// x = [ 0 0 ]
			x1 = new DMatrixRMaj(new double[][] {{ x},{vx  }});

			SimpleMatrix tmp = new SimpleMatrix(new double[][] {
				{ Math.pow(dt, 4d) / 4d, Math.pow(dt, 3d) / 2d },
				{ Math.pow(dt, 3d) / 2d, Math.pow(dt, 2d) } });
			// Q = [ dt^4/4 dt^3/2 ]
			//	     [ dt^3/2 dt^2   ]
			Qx = tmp.scale(Math.pow(accelNoise, 2)).getDDRM();
			// P0 = [ 1 1 ]
			//	      [ 1 1 ]
			P0x = new DMatrixRMaj(new double[][] { { 1, 1 }, { 1, 1 } });
			// R = [ measurementNoise^2 ]
			Rx = new SimpleMatrix(new double[][] {{ Math.pow(measurementNoise, 2) }}).getDDRM();

			// constant control input, increase velocity by 0.1 m/s per cycle
			u = new ArrayRealVector(new double[] { 0.0d });

			filterX = new KalmanFilterSimple();
		    filterX.configure(Ax, Qx, Hx); 

		    filterX.setState(x1, P0x);

		}
	}	
	class KalmanY {

		public KalmanY(double y, double vy) {
			// A = [ 1 dt ]
			//	     [ 0  1 ]
			Ay = new DMatrixRMaj(new double[][] { { 1, dt }, { 0, 1 } });
			// B = [ dt^2/2 ]
			//	     [ dt     ]
			B = new Array2DRowRealMatrix(new double[][] { { Math.pow(dt, 2d) / 2d }, { dt } });
			// H = [ 1 0 ]
			Hy = new DMatrixRMaj(new double[][] { { 1d, 0d } });
			// y = [ 0 0 ]
			y1 = new DMatrixRMaj(new double[][] {{ y },{vy }});

			SimpleMatrix tmp = new SimpleMatrix(new double[][] {
				{ Math.pow(dt, 4d) / 4d, Math.pow(dt, 3d) / 2d },
				{ Math.pow(dt, 3d) / 2d, Math.pow(dt, 2d) } });
			// Q = [ dt^4/4 dt^3/2 ]
			//	     [ dt^3/2 dt^2   ]
			Qy = tmp.scale(Math.pow(accelNoise, 2)).getDDRM();
			// P0 = [ 1 1 ]
			//	      [ 1 1 ]
			P0y = new DMatrixRMaj(new double[][] { { 1, 1 }, { 1, 1 } });
			// R = [ measurementNoise^2 ]
			Ry = new SimpleMatrix(new double[][] {{ Math.pow(measurementNoise, 2) }}).getDDRM();

			// constant control input, increase velocity by 0.1 m/s per cycle
			u = new ArrayRealVector(new double[] { 0.0d });

			filterY = new KalmanFilterSimple();
		    filterY.configure(Ay, Qy, Hy); 

		    filterY.setState(y1, P0y);

		}
	}

	RandomGenerator rand = new JDKRandomGenerator();

	RealVector tmpPNoise = new ArrayRealVector(new double[] { Math.pow(dt, 2d) / 2d, dt });
	RealVector mNoise = new ArrayRealVector(1);


	private void predictAndDraw(double x, double y, double t) {
		int fred = points.size() - 5;
		fred = (fred > 0)? fred: 0;
		Point3D point = new Point3D();
		point.x = x; point.y = y; point.t = t;
		points.add(point);
		vx = vy = 0.0;
		SimpleMatrix tmp = new SimpleMatrix(new double[][] {
			{ Math.pow(dt, 4d) / 4d, Math.pow(dt, 3d) / 2d },
			{ Math.pow(dt, 3d) / 2d, Math.pow(dt, 2d) } });
		Ax = new DMatrixRMaj(new double[][] { { 1, dt }, { 0, 1 } });
		Qx = tmp.scale(Math.pow(accelNoise, 2)).getDDRM();
		Hx = new DMatrixRMaj(new double[][] { { 1d, 0d } });
		Rx = new SimpleMatrix(new double[][] {{ Math.pow(measurementNoise, 2) }}).getDDRM();
		filterX.configure(Ax, Qx, Hx); 
		P0x = new DMatrixRMaj(new double[][] { { 1, 1 }, { 1, 1 } });
		//x1 = new DMatrixRMaj(new double[][] {{ points.get(fred).x }, { points.get(fred).t }});
		x1 = new DMatrixRMaj(new double[][] {{ points.get(fred).x }, { vx }});//?
	    filterX.setState(x1, P0x);
		Ay = new DMatrixRMaj(new double[][] { { 1, dt }, { 0, 1 } });
		Qy = tmp.scale(Math.pow(accelNoise, 2)).getDDRM();
		Hy = new DMatrixRMaj(new double[][] { { 1d, 0d } });
		Ry = new SimpleMatrix(new double[][] {{ Math.pow(measurementNoise, 2) }}).getDDRM();
		filterY.configure(Ay, Qy, Hy); 
		P0y = new DMatrixRMaj(new double[][] { { 1, 1 }, { 1, 1 } });
		//y1 = new DMatrixRMaj(new double[][] {{ points.get(fred).y }, { points.get(fred).t  }});
		y1 = new DMatrixRMaj(new double[][] {{ points.get(fred).y }, { vy  }});//?
	    filterY.setState(y1, P0y);
		
		// Collect data.
		for (int i = 0; i < 5 && i < points.size(); i++) {
			fred = points.size() - 5+i;
			fred = (fred > 0)? fred: 0;
			filterX.predict();

			// x = A * y + B * u + pNoise
			//x1 = A.operate((new ArrayRealVector(new double[] {points.get(i).x,points.get(i).t}))).add(B.operate(u)).add(pNoise);
			x1 = new SimpleMatrix(new double[][] {{points.get(fred).x}}).getDDRM();

			// z = H * y + m_noise
			DMatrixRMaj z = x1;

	
			filterX.update(z, Rx);

		}


		// Collect data.
		for (int i = 0; i < 5 && i < points.size(); i++) {
			fred = points.size() - 5+i;
			fred = (fred > 0)? fred: 0;
			filterY.predict();

			// y = A * y + B * u + pNoise
			//y1 = A.operate((new ArrayRealVector(new double[] {points.get(i).y,points.get(i).t}))).add(B.operate(u)).add(pNoise);
			y1 = new SimpleMatrix(new double[][] {{points.get(fred).y}}).getDDRM();

			// z = H * y + m_noise
			DMatrixRMaj z = y1;

	
			filterX.update(z, Ry);

		}


		Point3D ppoint = new Point3D();
		double positionX = filterX.getState().get(0, 0);
		double velocityX = filterX.getState().get(1, 0);
		double positionY = filterY.getState().get(0, 0);
		double velocityY = filterY.getState().get(1, 0);
		System.out.println("velocityX:"+velocityX);
		ppoint.x = positionX; ppoint.y = positionY; ppoint.t = t;		
		ppoints.add(ppoint);

	}	

//	private void predictAndDraw(double x, double y, double t) {
//		Point3D point = new Point3D();
//		point.x = x; point.y = y; point.t = t;
//		points.add(point);
//		// Collect data.
//		final WeightedObservedPoints obs = new WeightedObservedPoints();
//		for (int i = 0; i < 5 && i < points.size(); i++) {
//			int fred = points.size() - 5 +i;
//			fred = (fred > 0)? fred: 0;
//			obs.add(points.get(fred).t, points.get(fred).x);
//
//		}
//
//		// Instantiate a third-degree polynomial fitter.
//		final PolynomialCurveFitter fitter = PolynomialCurveFitter.create(1);
//
//		// Retrieve fitted parameters (coefficients of the polynomial function).
//		final double[] coeff = fitter.fit(obs.toList());
//		// Collect data.
//		final WeightedObservedPoints obs2 = new WeightedObservedPoints();
//		for (int i = 0; i < 5 && i < points.size(); i++) {
//			int fred = points.size() - 5+i;
//			fred = (fred > 0)? fred: 0;
//			obs2.add(points.get(fred).t, points.get(fred).y);
//
//		}
//
//		// Instantiate a third-degree polynomial fitter.
//		final PolynomialCurveFitter fitter2 = PolynomialCurveFitter.create(1);
//
//		// Retrieve fitted parameters (coefficients of the polynomial function).
//		final double[] coeff2 = fitter2.fit(obs2.toList());
//		Point3D ppoint = new Point3D();
//		ppoint.x = predicted(t, coeff); ppoint.y = predicted(t, coeff2); ppoint.t = t;		
//		ppoints.add(ppoint);
//
//	}
	double predicted(double t, double[] c) {
		double sumOfTerms = 0;
		for (int pow = 0; pow < c.length; pow++) {
			sumOfTerms += Math.pow(t, pow)*c[pow];
		}
		return sumOfTerms;
	}
	double addNoise(double noiseless) {
		return NOISE_VALUE * (Math.random()-0.5)+noiseless;
	}
	class PaintDemo{



		PaintDemo(){
			jfrm.setSize(1000, 800);
			jfrm.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

			doRender.addActionListener(new ActionListener() {

				@Override
				public void actionPerformed(ActionEvent e) {
					render = true;
					diamond.repaint();				
				}

			});
			jfrm.setLayout(new BorderLayout());
			//			canvas.setSize(1000, 750);
			//			canvas.setVisible(true);
			JPanel temp = new JPanel();
			temp.add(maxxl);		
			temp.add(maxx);
			temp.add(doRender);
			//			canvas.setVisible(true);
			jfrm.add(temp, BorderLayout.NORTH);
			jfrm.add(diamond, BorderLayout.CENTER);

			//jfrm.add(diamond);

			jfrm.setVisible(true);
		}

	}
	public static void main(String args[]) {
		SwingUtilities.invokeLater(new Runnable() {
			public void run() {
				diamond = new LinearFit();
				diamond.new PaintDemo();
				diamond.new KalmanX(0.0,0.0);
				diamond.new KalmanY(0.0,0.0);
				try {
					diamond.doASquare();
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}

			}
		});

	}

}
