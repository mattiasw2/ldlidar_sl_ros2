# map_sheet.py OUT.png LABEL=MAP.pgm [LABEL=MAP.pgm ...]
# Side-by-side contact sheet of occupancy grids, each cropped to its known area.
import sys
from PIL import Image, ImageDraw, ImageOps
items=[a.split('=',1) for a in sys.argv[2:]]
ims=[]
for label,p in items:
    im=Image.open(p).convert('L')
    bb=ImageOps.invert(im.point(lambda v: 0 if v==205 else 255)).getbbox() or (0,0,*im.size)
    im=im.crop(bb); ims.append((label,im))
S=360
tiles=[]
for label,im in ims:
    im=im.copy(); im.thumbnail((S,S),Image.NEAREST) if max(im.size)>S else None
    sc=min(S/im.width,S/im.height); im=im.resize((int(im.width*sc),int(im.height*sc)),Image.NEAREST)
    t=Image.new('L',(S,S+24),255); t.paste(im,((S-im.width)//2,24)); ImageDraw.Draw(t).text((6,6),label,fill=0); tiles.append(t)
cols=3; rows=(len(tiles)+cols-1)//cols
out=Image.new('L',(cols*(S+8),rows*(S+32)),255)
for i,t in enumerate(tiles): out.paste(t,((i%cols)*(S+8),(i//cols)*(S+32)))
out.save(sys.argv[1])
