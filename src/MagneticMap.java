import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Container;
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
public class MagneticMap extends JPanel{
	class Point3D{
		public double x,y,t;
	}
	static RandomGenerator rand = new JDKRandomGenerator();
	float[][] magmap = {{50f,49f,48f,42f},
			{51f,50f,48f,43f},
			{50f,50f,49f,47f},
			{52f,50f,46f,57f},
			{52f,49f,47f,48f},
			{49f,50f,50f,48f},
			{48f,48f,49f,49f},
			{47f,47f,49f,43f},
			{51f,48f,47f,40f},
			{53f,53f,60f,49f}};
	private static double NOISE_VALUE = 2.0;
	List<Point3D> points;
	List<Point3D> ppoints;
	List<Point3D> vppoints;
	protected static boolean render = true;
	JLabel minxl= new JLabel("Noise");;
	JTextField minx= new JTextField("2",20);;
	JLabel maxxl = new JLabel("Scale");;
	JTextField maxx =new JTextField("60",20);
	JButton doRender= new JButton("Render");
	JPanel canvas = new JPanel();
	JFrame jfrm = new JFrame("Kalman Filter");
	static MagneticMap charlie;
	MagneticMap(){	
		setBorder(BorderFactory.createLineBorder(Color.BLACK, 4));
	}

	@Override
	protected void paintComponent(Graphics g) {
		super.paintComponent(g);
		int height = getHeight();
		int width = getWidth();
		float max, min;
		max = min = 0.0f;
		for (int y = 0; y < magmap.length; y++) {
			for (int x = 0; x < magmap[y].length; x++){
				min = (magmap[y][x] < min)? magmap[y][x]: min;
				max = (magmap[y][x] > max)? magmap[y][x]: max;			
			}
		}
		float range = max - min;
		int max_x = Integer.parseInt(this.maxx.getText());
		for (int y = 0; y < magmap.length; y++) {
			for (int x = 0; x < magmap[y].length; x++){
				
				g.setColor(new Color((1.0f/range) * (magmap[y][x]-min), 0.0f,0.0f));
				g.fillRect(x*max_x, y*max_x, max_x, max_x);	
				g.setColor(Color.BLACK);
				g.drawString(""+magmap[y][x]+"uT", x*max_x, y*max_x+max_x);
				g.drawRect(x*max_x, y*max_x, max_x, max_x);
			}
		}
		

	}


	class PaintDemo{



		PaintDemo(){
			jfrm.setSize(1000, 800);
			jfrm.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

			doRender.addActionListener(new ActionListener() {

				@Override
				public void actionPerformed(ActionEvent e) {
					render = true;
					NOISE_VALUE = Double.parseDouble(minx.getText());

					charlie.repaint();				
				}

			});
			jfrm.setLayout(new BorderLayout());
			//			canvas.setSize(1000, 750);
			//			canvas.setVisible(true);
			JPanel temp = new JPanel();
			temp.add(maxxl);		
			temp.add(maxx);
			temp.add(minxl);		
			temp.add(minx);
			temp.add(doRender);
			//			canvas.setVisible(true);
			jfrm.add(temp, BorderLayout.NORTH);
			jfrm.add(charlie, BorderLayout.CENTER);

			//jfrm.add(diamond);

			jfrm.setVisible(true);
		}

	}
	public static void main(String args[]) {
		SwingUtilities.invokeLater(new Runnable() {
			public void run() {
				charlie = new MagneticMap();
				charlie.new PaintDemo();

			}
		});

	}

}
