import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.SwingUtilities;

import org.apache.commons.math3.random.JDKRandomGenerator;
import org.apache.commons.math3.random.RandomGenerator;


@SuppressWarnings("serial")
public class MagneticMap extends JPanel{
	class Point3D{
		public double x,y,t;
	}
	static RandomGenerator rand = new JDKRandomGenerator();
//	float[][] magmap = {{50f,49f,48f,42f},
//	{51f,50f,48f,43f},
//	{50f,50f,49f,47f},
//	{52f,50f,46f,57f},
//	{52f,49f,47f,48f},
//	{49f,50f,50f,48f},
//	{48f,48f,49f,49f},
//	{47f,47f,49f,43f},
//	{51f,48f,47f,40f},
//	{53f,53f,60f,49f}};
//	float[][] magmapxy = {{7f,10f,9f,22f},
//	{7f,18f,4f,20f},
//	{7f,21f,6f,11f},
//	{10f,27f,9f,10f},
//	{13f,23f,7f,18f},
//	{14f,13f,11f,16f},
//	{6f,8f,7f,9f},
//	{10f,5f,4f,10f},
//	{12f,2f,6f,17f},
//	{19f,7f,8f,17f}};
//	
//	float[][] magmapz = {{46f,47f,42f,36f},
//	{43f,45f,45f,43f},
//	{42f,44f,45f,56f},
//	{41f,42f,41f,46f},
//	{40f,45f,43f,39f},
//	{39f,48f,42f,41f},
//	{45f,47f,44f,44f},
//	{47f,45f,43f,42f},
//	{41f,46f,43f,40f},
//	{48f,45f,51f,43f}};
	float[][] magmapx = {{0f,8f,-9f,0f},
	{-1f,7f,-7f,-4f},
	{0f,6f,-9f,-4f},
	{1f,8f,-10f,0f},
	{6f,17f,-9f,-2f},
	{-6f,16f,-12f,-7f},
	{-11f,7f,-8f,-6f},
	{2f,2f,-6f,-4f},
	{2f,0f,-5f,4f},
	{7f,4f,2f,-4f}};
	float[][] magmapy = {{23f,22f,20f,24f},
	{20f,22f,18f,27f},
	{21f,21f,10f,17f},
	{18f,19f,18f,12f},
	{18f,20f,17f,25f},
	{11f,21f,13f,22f},
	{16f,19f,11f,19f},
	{11f,13f,12f,8f},
	{19f,17f,12f,16f},
	{5f,19f,6f,14f}};	
	float[][] magmapz = {{-40f,-38f,-37f,-34f},
	{-41f,-39f,-39f,-39f},
	{-41f,-39f,-42f,-57f},
	{-41f,-39f,-38f,-42f},
	{-41f,-35f,-40f,-35f},
	{-43f,-36f,-42f,-37f},
	{-42f,-40f,-42f,-29f},
	{-40f,-39f,-40f,-32f},
	{-45f,-39f,-41f,-39f},
	{-52f,-39f,-51f,-39f}};

	
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
	JFrame jfrm = new JFrame("Magnetic Map ");
	static MagneticMap charlie;
	MagneticMap(){	
		setBorder(BorderFactory.createLineBorder(Color.BLACK, 4));
	}

	@Override
	protected void paintComponent(Graphics g) {
		super.paintComponent(g);
		int height = getHeight();
		int width = getWidth();
		float maxx, minx;
		maxx = minx = 0.0f;
		for (int y = 0; y < magmapx.length; y++) {
			for (int x = 0; x < magmapx[y].length; x++){
				minx = (magmapx[y][x] < minx)? magmapx[y][x]: minx;
				maxx = (magmapx[y][x] > maxx)? magmapx[y][x]: maxx;			
			}
		}
		float rangex = maxx - minx;
		float maxy,miny;
		maxy = miny = 0.0f;
		for (int y = 0; y < magmapy.length; y++) {
			for (int x = 0; x < magmapy[y].length; x++){
				miny = (magmapy[y][x] < miny)? magmapy[y][x]: miny;
				maxy = (magmapy[y][x] > maxy)? magmapy[y][x]: maxy;			
			}
		}
		float rangey = maxy - miny;
		float maxz,minz;
		maxz = minz = 0.0f;
		for (int y = 0; y < magmapz.length; y++) {
			for (int x = 0; x < magmapz[y].length; x++){
				minz = (magmapz[y][x] < minz)? magmapz[y][x]: minz;
				maxz = (magmapz[y][x] > maxz)? magmapz[y][x]: maxz;			
			}
		}
		float rangez = maxz - minz;		
		float max,min;
		max = min = 0.0f;
		for (int y = 0; y < magmapz.length; y++) {
			for (int x = 0; x < magmapz[y].length; x++){
				float magnitude = (float)Math.sqrt((double)(magmapx[y][x]*magmapx[y][x]+magmapy[y][x]*magmapy[y][x]+magmapz[y][x]*magmapz[y][x]));
				min = (magnitude < min)? magnitude: min;
				max = (magnitude > max)? magnitude: max;			
			}
		}
		float range = max - min;
		int max_x = Integer.parseInt(this.maxx.getText());
		for (int y = 0; y < magmapx.length; y++) {
			for (int x = 0; x < magmapx[y].length; x++){
				float magnitude = (float)Math.sqrt((double)(magmapx[y][x]*magmapx[y][x]+magmapy[y][x]*magmapy[y][x]+magmapz[y][x]*magmapz[y][x]));

				g.setColor(new Color((1.0f/range) * (magnitude-min), 0.0f,0.0f));
				g.fillRect(x*max_x, y*max_x, max_x, max_x);	
				g.setColor(Color.WHITE);
				g.drawString(""+Math.round(magnitude)+"uT m", x*max_x, y*max_x+max_x);
				g.drawRect(x*max_x, y*max_x, max_x, max_x);
			}
		}

		max_x = Integer.parseInt(this.maxx.getText());
		for (int y = 0; y < magmapx.length; y++) {
			for (int x = 0; x < magmapx[y].length; x++){
				
				g.setColor(new Color((1.0f/rangex) * (magmapx[y][x]-minx), 
						(1.0f/rangey) * (magmapy[y][x]-miny),(1.0f/rangez) * (magmapz[y][x]-minz)));
				g.fillRect(x*max_x+width/2, y*max_x, max_x, max_x);	
				g.setColor(Color.BLACK);
				g.drawString(""+Math.round(magmapx[y][x])+","+Math.round(magmapy[y][x])+","+Math.round(magmapz[y][x]), x*max_x+width/2, y*max_x+max_x);
				g.drawRect(x*max_x+width/2, y*max_x, max_x, max_x);
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
