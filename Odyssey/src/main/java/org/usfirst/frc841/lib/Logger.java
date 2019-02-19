package org.usfirst.frc841.lib;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

public class Logger {
	public Logger(){
		try {
			File f = new File("/home/lvuser/data.txt");
			f.createNewFile();
			FileOutputStream oFile = new FileOutputStream(f, false);
			oFile.write("Started \r".getBytes());
			oFile.flush();
			oFile.close();
		}
		catch(IOException e) {
			System.out.println("error:" + e.getMessage());
		}
	}
	
	public void write(String content){
		String txt = content + "\r";
		try {
			File f = new File("/home/lvuser/data.txt");
			f.createNewFile();
			FileOutputStream oFile = new FileOutputStream(f, true);
			oFile.write(txt.getBytes());
			oFile.flush();
			oFile.close();
		}
		catch(IOException e) {
			System.out.println("error:" + e.getMessage());
		}
	}
	


}
