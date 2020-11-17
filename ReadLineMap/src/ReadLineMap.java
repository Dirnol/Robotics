import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;

import javax.xml.stream.XMLStreamException;

import lejos.robotics.geometry.Line;
import lejos.robotics.geometry.Rectangle;
import lejos.robotics.mapping.LineMap;
import lejos.robotics.mapping.SVGMapLoader;

public class ReadLineMap {

	public static void main(String[] args) {
		new ReadLineMap();
	}
	
	public ReadLineMap() {
		try {
			FileInputStream fStream = new FileInputStream(new File("map.svg"));
			SVGMapLoader mapLoader = new SVGMapLoader(fStream);
			LineMap map = mapLoader.readLineMap();
			Line[] lines = map.getLines();
			System.out.println(lines.length);
			for(Line l : lines) {
				System.out.println("(" + l.x1 + ", " + l.y1 + ") to (" + l.x2 + ", " + l.y2 + ")");
			}
			printRect(map.getBoundingRect());
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (XMLStreamException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	private void printRect(Rectangle rect) {
		System.out.println("(" + rect.getMinX() + ", " + rect.getMinY() + ") x (" + rect.getMaxX() + ", " + rect.getMaxY() + ")");
	}
	
}
