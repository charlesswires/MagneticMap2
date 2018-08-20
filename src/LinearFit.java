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

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.random.JDKRandomGenerator;
import org.apache.commons.math3.random.RandomGenerator;


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
	// discrete time interval
	double dt = 1d;
	// position measurement noise (meter)
	double measurementNoise = 2d;
	// acceleration noise (meter/sec^2)
	double accelNoise = 0.0d;
	private KalmanFilter filterX;
	private KalmanFilter filterY;
	RealVector u;
	RealVector x1;
	RealVector y1;
	RealMatrix A;
	RealMatrix B;
	RealMatrix H;
	class KalmanX {
		public KalmanX(double x, double t) {
			// A = [ 1 dt ]
			//	     [ 0  1 ]
			A = new Array2DRowRealMatrix(new double[][] { { 1, dt }, { 0, 1 } });
			// B = [ dt^2/2 ]
			//	     [ dt     ]
			B = new Array2DRowRealMatrix(new double[][] { { Math.pow(dt, 2d) / 2d }, { dt } });
			// H = [ 1 0 ]
			H = new Array2DRowRealMatrix(new double[][] { { 1d, 0d } });
			// x = [ 0 0 ]
			x1 = new ArrayRealVector(new double[] { x, t });

			RealMatrix tmp = new Array2DRowRealMatrix(new double[][] {
				{ Math.pow(dt, 4d) / 4d, Math.pow(dt, 3d) / 2d },
				{ Math.pow(dt, 3d) / 2d, Math.pow(dt, 2d) } });
			// Q = [ dt^4/4 dt^3/2 ]
			//	     [ dt^3/2 dt^2   ]
			RealMatrix Q = tmp.scalarMultiply(Math.pow(accelNoise, 2));
			// P0 = [ 1 1 ]
			//	      [ 1 1 ]
			RealMatrix P0 = new Array2DRowRealMatrix(new double[][] { { 1, 1 }, { 1, 1 } });
			// R = [ measurementNoise^2 ]
			RealMatrix R = new Array2DRowRealMatrix(new double[] { Math.pow(measurementNoise, 2) });

			// constant control input, increase velocity by 0.1 m/s per cycle
			u = new ArrayRealVector(new double[] { 0.0d });

			ProcessModel pm = new DefaultProcessModel(A, B, Q, x1, P0);
			MeasurementModel mm = new DefaultMeasurementModel(H, R);
			filterX = new KalmanFilter(pm, mm);
		}
	}	
	class KalmanY {
		public KalmanY(double y, double t) {
			// A = [ 1 dt ]
			//	     [ 0  1 ]
			A = new Array2DRowRealMatrix(new double[][] { { 1, dt }, { 0, 1 } });
			// B = [ dt^2/2 ]
			//	     [ dt     ]
			B = new Array2DRowRealMatrix(new double[][] { { Math.pow(dt, 2d) / 2d }, { dt } });
			// H = [ 1 0 ]
			H = new Array2DRowRealMatrix(new double[][] { { 1d, 0d } });
			// x = [ 0 0 ]
			y1 = new ArrayRealVector(new double[] { y, t });

			RealMatrix tmp = new Array2DRowRealMatrix(new double[][] {
				{ Math.pow(dt, 4d) / 4d, Math.pow(dt, 3d) / 2d },
				{ Math.pow(dt, 3d) / 2d, Math.pow(dt, 2d) } });
			// Q = [ dt^4/4 dt^3/2 ]
			//	     [ dt^3/2 dt^2   ]
			RealMatrix Q = tmp.scalarMultiply(Math.pow(accelNoise, 2));
			// P0 = [ 1 1 ]
			//	      [ 1 1 ]
			RealMatrix P0 = new Array2DRowRealMatrix(new double[][] { { 1, 1 }, { 1, 1 } });
			// R = [ measurementNoise^2 ]
			RealMatrix R = new Array2DRowRealMatrix(new double[] { Math.pow(measurementNoise, 2) });

			// constant control input, increase velocity by 0.1 m/s per cycle
			u = new ArrayRealVector(new double[] { 0.0d });

			ProcessModel pm = new DefaultProcessModel(A, B, Q, y1, P0);
			MeasurementModel mm = new DefaultMeasurementModel(H, R);
			filterY = new KalmanFilter(pm, mm);
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
		KalmanX Kx = new KalmanX(points.get(fred).x,points.get(fred).t);KalmanY Ky = new KalmanY(points.get(fred).y,points.get(fred).t);
		// Collect data.
		for (int i = 0; i < 5 && i < points.size(); i++) {
			fred = points.size() - 5+i;
			fred = (fred > 0)? fred: 0;
			filterX.predict(u);

			// x = A * y + B * u + pNoise
			//x1 = A.operate((new ArrayRealVector(new double[] {points.get(i).x,points.get(i).t}))).add(B.operate(u)).add(pNoise);
			x1 = new ArrayRealVector(new double[] {points.get(fred).x,points.get(fred).t});

			// z = H * y + m_noise
			RealVector z = H.operate(x1).add(mNoise);

	
			filterX.correct(z);

		}


		// Collect data.
		for (int i = 0; i < 5 && i < points.size(); i++) {
			fred = points.size() - 5+i;
			fred = (fred > 0)? fred: 0;
			filterY.predict(u);

			// y = A * y + B * u + pNoise
			//y1 = A.operate((new ArrayRealVector(new double[] {points.get(i).y,points.get(i).t}))).add(B.operate(u)).add(pNoise);
			y1 = new ArrayRealVector(new double[] {points.get(fred).y,points.get(fred).t});

			// z = H * y + m_noise
			RealVector z = H.operate(y1).add(mNoise);

			filterY.correct(z);

		}


		Point3D ppoint = new Point3D();
		double positionX = filterX.getStateEstimation()[0];
		double velocityX = filterX.getStateEstimation()[1];
		double positionY = filterY.getStateEstimation()[0];
		double velocityY = filterY.getStateEstimation()[1];
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
