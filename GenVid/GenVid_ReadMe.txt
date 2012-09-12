
GenVid Command Line Manual 

Required Arguments:
'arg1' : Video path as first argument. 
'arg2' :  Tracking data directory as second argument.
'arg3' :  Save path including file name as third argument.


Text Flags	
	'-id' : turns on ant label for export.
	'-xy' : turns on ant XY position text for export.
	'-ang' : turns on ant angle text for export.

	'-s'	: Small Font flag, makes font size small. 	
	'-l'	: Large Font flag, makes font size large. 	


Vis Flags
	'-cir' : Circle flag turns on circles for export.
		Optional flags follow Circle flag:
		'-crad' 'int' : radius flag followed by integer adjusts radius of circle.
		'-cs' 'int' : Circle Stroke flag followed by integer adjusts stroke of circle.
		'-ccol' : Turns on uniquely colored circles for export.

	'-tr' : Trail flag turns on trails for export.
		Optional flags follow Trail flag:
		'-tsize' 'int' : Trail Size flag followed by integer adjusts length of trail.
		'-trs' 'int' : Circle Stroke flag followed by integer adjusts stroke of trail.
		'-tcol' : Turns on uniquely colored trails for export.

	'-ar' : Arrow flag turns on orientation arrows for export.
		Optional flags follow Arrow flag:
		'-asize' 'int' : Arrow Size flag followed by integer adjusts length of arrow.
		'-ars' 'int' : Arrow Stroke flag followed by integer adjusts stroke of arrow.
		'-acol' : Turns on uniquely colored arrows for export.		

	'-box' turns on boxes for export.
		Options:
		'-box' 'int' 'int' 	: Box flag followed by width & height adjusts width & height of box.



-id
-xy
-ang
-s
-l
-cir
	-rad
	-cs
	-ccol
-tr
	-tsize
	-trs
	-tcol
-ar
	-asize
	-ars
	-acol
-box


-id
-xy
-ang
-s
-l
-cir
-tr
-ar
-box