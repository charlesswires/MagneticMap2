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
	float[][] magmapxy = {{7f,10f,9f,22f},
	{7f,18f,4f,20f},
	{7f,21f,6f,11f},
	{10f,27f,9f,10f},
	{13f,23f,7f,18f},
	{14f,13f,11f,16f},
	{6f,8f,7f,9f},
	{10f,5f,4f,10f},
	{12f,2f,6f,17f},
	{19f,7f,8f,17f}};
	
	float[][] magmapz = {{46f,47f,42f,36f},
	{43f,45f,45f,43f},
	{42f,44f,45f,56f},
	{41f,42f,41f,46f},
	{40f,45f,43f,39f},
	{39f,48f,42f,41f},
	{45f,47f,44f,44f},
	{47f,45f,43f,42f},
	{41f,46f,43f,40f},
	{48f,45f,51f,43f}};
	
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
		float max, min;
		max = min = 0.0f;
		for (int y = 0; y < magmapxy.length; y++) {
			for (int x = 0; x < magmapxy[y].length; x++){
				min = (magmapxy[y][x] < min)? magmapxy[y][x]: min;
				max = (magmapxy[y][x] > max)? magmapxy[y][x]: max;			
			}
		}
		float range = max - min;
		int max_x = Integer.parseInt(this.maxx.getText());
		for (int y = 0; y < magmapxy.length; y++) {
			for (int x = 0; x < magmapxy[y].length; x++){
				
				g.setColor(new Color((1.0f/range) * (magmapxy[y][x]-min), 0.0f,0.0f));
				g.fillRect(x*max_x, y*max_x, max_x, max_x);	
				g.setColor(Color.WHITE);
				g.drawString(""+magmapxy[y][x]+"uT xy", x*max_x, y*max_x+max_x);
				g.drawRect(x*max_x, y*max_x, max_x, max_x);
			}
		}
		max = min = 0.0f;
		for (int y = 0; y < magmapz.length; y++) {
			for (int x = 0; x < magmapz[y].length; x++){
				min = (magmapz[y][x] < min)? magmapz[y][x]: min;
				max = (magmapz[y][x] > max)? magmapz[y][x]: max;			
			}
		}
		range = max - min;
		max_x = Integer.parseInt(this.maxx.getText());
		for (int y = 0; y < magmapz.length; y++) {
			for (int x = 0; x < magmapz[y].length; x++){
				
				g.setColor(new Color((1.0f/range) * (magmapz[y][x]-min), 0.0f,0.0f));
				g.fillRect(x*max_x+width/2, y*max_x, max_x, max_x);	
				g.setColor(Color.WHITE);
				g.drawString(""+magmapz[y][x]+"uT z", x*max_x+width/2, y*max_x+max_x);
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
