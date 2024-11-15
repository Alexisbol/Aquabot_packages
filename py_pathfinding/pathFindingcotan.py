


import csv
from collections import deque
import math as ma
import matplotlib.pyplot as plt
import copy
import numpy as np
from math import *



def rad(a):
    return(ma.pi*a/180)





def sommetdansgraphe (s,g):
    if s in g :
        return(True)
    else :
        return(False)





from random import randint
from random import choice


def ext_comp(G,cle):
    gext = {}
    for i in G:
        gext[i]=G[i][cle]
    return gext




def convert_tot(dXY):
    x,y = [],[]
    for v in dXY.values():
        x.append(v[0])
        y.append(v[1])
    return (x,y)



def tracecol(g,l=[]):
    i=0
    fig, ax = plt.subplots(figsize = (15,7))

    for a in g :
        i+=1
        for b in g[a]["adj"]:
            xa,ya=a
            xb,yb=b
            if g[a]["rap"] and g[b]["rap"]:
                ax.plot([xa,xb], [ya,yb], color="red")
            else :
                ax.plot([xa,xb], [ya,yb], color="blue")
    for i in range(len(l)-1):
        xa,ya=l[i]
        xb,yb=l[i+1]
        ax.plot([xa,xb], [ya,yb], color="yellow")

    plt.show()



def estconnexe (g):
    d={}
    lr=[]
    for e in g:
        d[e]=False

        lr.append(e)
    r=choice(lr)
    l=[r]
    while len(l)>0 :
        for v in g[l[0]]["adj"]:
            if v in g :

                if d[v]==False :
                    l.append(v)
            d[l[0]]=True
        l.remove(l[0])


    gr=copy.deepcopy(g)
    for e in g :
        if d[e]==False:
            del gr[e]
    tracegr(gr)

    for e in d :

        if d[e] == False :
            #print(e)
            return(False)
    return(True)

def compconnexe(g):
    d={}
    lr=[]
    for e in g:
        d[e]=False

        lr.append(e)
    r=choice(lr)
    l=[r]



    while len(l)>0 :
        for v in g[l[0]]["adj"]:
            if v in g :

                if d[v]==False :
                    l.append(v)
            d[l[0]]=True
        l.remove(l[0])
    gr={}
    for e in g :
        if d[e]==True :
            gr[e]=g[e]
    return(gr)




def nbinter (g):
    d={}
    for e in g :
        l=len(g[e]["adj"])
        if l in  d :
            d[l]+=1
        else :
            d[l]=1
    return(d)





def dijkstra (G,r):

    g=initp(G)
    g[r]["e"]=0
    s=[]
    for i in range(len(g)):

        s.append(plusfaible (g))
        for e in g[s[-1]]["adj"]:

            if g[e]["e"]<0 or g[e]["e"]> g[s[-1]]["adj"][e]+g[s[-1]]["e"] :

                g[e]["e"]= g[s[-1]]["adj"][e]+g[s[-1]]["e"]
                g[e]["parent"]=s[-1]
            else :

                ()
        g[s[-1]]["vu"]=1
    return (g)



def diststar(G,r,a,c=0.1):
    #print("r=",r," a=",a)
    g=initp(G)
    g[r]["e"]=0
    s=[r]
    while g[a]["vu"]==0 :
        m=ma.inf
        for e in s :
            if g[e]["vu"]==0:
                if  "h" not in g[e]:
                    g[e]["h"]=dist(e,a)
                if g[e]["e"]-c*g[e]["h"]<m:
                    m=g[e]["e"]-c*g[e]["h"]
                    x=e
        for v in g[x]["adj"]:
            p=g[x]["adj"][v][0]+g[x]["e"]
            if g[v]["vu"]==0 and ( ( "e" not in g[v]) or g[v]["e"]>p ) :
                g[v]["e"]=p
                g[v]["p"]=x
                if v not in s:
                    s.append(v)

        s.remove(x)
        g[x]["vu"]=1
    l=[a]
    pt=0
    while l[-1] != r :
        l.append(g[l[-1]]["p"])
        pt+=g[l[-2]]["adj"][l[-1]][0]
    return(l,pt)



def initp (g):
    gr = copy.deepcopy(g)
    for e in gr :
        gr[e]["vu"]=0
        gr[e]["e"]=ma.inf
    return (gr)

def distance (G,r,a):

    g=initp(G)
    g[r]["e"]=0
    s=[]
    while g[a]["vu"]==0  :
        s.append(plusfaible (g))
        for e in g[s[-1]]["adj"]:
            if g[e]["e"]<0 or g[e]["e"]> g[s[-1]]["adj"][e]+g[s[-1]]["e"] :
                g[e]["e"]= g[s[-1]]["adj"][e]+g[s[-1]]["e"]
                g[e]["parent"]=s[-1]
            else :
                ()
        g[s[-1]]["vu"]=1
    l=[a]
    while l[-1]!=r:
        l.append(g[l[-1]]["parent"])
    return (l)

from random import *

def soit(g):
    l=[]
    for e in g :
        l.append(e)
    return(choice(l))








def plusfaible (g):
    m=-1
    s=[]
    for e in g:
        if g[e]["e"]>=0 and (g[e]["e"]<m or m<0) and g[e]["vu"]==0:
            m=g[e]["e"]
            s.append(e)
        else :
            ()
    return (s[-1])


##



def heur (g,a="A55"):
    for e in g :
        g[e]["h"]=dist(e,a)

def pdist (g,c,r):
    heur (g,r)
    for e in g :
        for s in g[e]["adj"]:
            g[e]["adj"][s]=g[e]["adj"][s]-c*(g[e]["h"]-g[s]["h"])


def astar (gr,r,t,c=0.9):
    P = copy.deepcopy(gr)
    pdist(P,c,r)
    return (distance(P,r,t))









import random

# Fonction pour générer une couleur aléatoire en format RGB
def couleur_aleatoire():
    return (random.random(), random.random(), random.random())

def tracer_cercle(centre, rayon,c):
    theta = np.linspace(0, 2 * np.pi, int(rayon*2*np.pi))  
    x = centre[0] + rayon * np.cos(theta)
    y = centre[1] + rayon * np.sin(theta)
    plt.plot(x, y,c)
    

def dist(A,B):
    return sqrt((A[0]-B[0])**2+(A[1]-B[1])**2)

def tracer_ligne(a,b,c):
    plt.plot((a[0],b[0]),(a[1],b[1]),c)
    
def pointscotan(A,B,l,liobs):
    [a,ra]=A
    [b,rb]=B
    [xa,ya]=a 
    [xb,yb]=b 

    if (ra+rb)/dist(a,b)<1:
        tet=acos((ra+rb)/dist(a,b))
        gam=atan2(yb-ya,xb-xa)

        c=(xa+ra*cos(gam+tet),ya+ra*sin(gam+tet))
        d=(xa+ra*cos(gam-tet),ya+ra*sin(gam-tet))
        e=(xb-rb*cos(gam+tet),yb-rb*sin(gam+tet))
        f=(xb-rb*cos(gam-tet),yb-rb*sin(gam-tet))

        ajoute(c,e,A,B,liobs,l)
        ajoute(d,f,A,B,liobs,l)
        
        
    inv = False
    if rb>ra :
        A,B=B,A
        inv = True
    
    tet=acos((ra-rb)/dist(a,b))
    gam=atan2(yb-ya,xb-xa)

    c=(xa+ra*cos(gam+tet),ya+ra*sin(gam+tet))
    d=(xa+ra*cos(gam-tet),ya+ra*sin(gam-tet))
    e=(xb+rb*cos(gam+tet),yb+rb*sin(gam+tet))
    f=(xb+rb*cos(gam-tet),yb+rb*sin(gam-tet))

    ajoute(c,e,A,B,liobs,l)
    ajoute(d,f,A,B,liobs,l)

    if inv :
      A,B=B,A  
    
    

def ajoute(a,b,c1,c2,liobs,llignes):
    if verif(a,b,c1,c2,liobs):
        llignes.append((a,c1,b,c2))



def sommet(G,s,c):
    """Ajoute le sommet s au graphe G"""
    if s not in G:
        G[s]={"adj":{},"centre":c}

def arretedroite(G,s,t):
    """Ajoute au graphe G une arete de poids p entre les sommets s et t"""
    d=dist(s,t)
    G[s]["adj"][t]=[d]
    G[t]["adj"][s]=[d]

def angle(p,c):
    return atan2(p[1]-c[1],p[0]-c[0])


def arretecourbe(G,s,t,c):
    tet = angle(s,c[0])-angle(t,c[0])
    if tet > pi :
        tet = 2*pi - tet
    d = abs(tet)*c[1]

    G[s]["adj"][t]=[d,c]
    G[t]["adj"][s]=[d,c]



def verif(a,b,c1,c2,liobs):
    xa,ya=a
    xb,yb=b
    color=couleur_aleatoire()
    for (c,rc) in liobs:
        if True or((c,rc)!=c1 and (c,rc)!=c2) :
            if dist(a,c)<rc-1 or dist(b,c)<rc-1 :
                #tracer_cercle(c,rc,color)
                #tracer_ligne(a,b,color)
                return False
            xc,yc=c
            u=((xc-xa)*(xb-xa)+(yc-ya)*(yb-ya))/(dist(a,b)**2)
            if u>0 and u<1 :
                p=[xa+u*(xb-xa),ya+u*(yb-ya)]
                if dist(p,c)<rc-1:
                    return False
    return True



"""def tracebrut():

    plt.figure(figsize=(6, 6))
    for centre, rayon in Liobs:
        tracer_cercle(centre, rayon,'b')

    for (a,c1,b,c2) in llignes:
        tracer_ligne(a,b,'b')
    

    # Personnalisation du graphique
    plt.gca().set_aspect('equal', adjustable='box')  # Garder les proportions des cercles
    plt.grid(False)
    plt.xlabel('X')
    plt.ylabel('Y')
    #plt.title('Cercles avec centres et rayons donnés')
    #plt.legend()

    # Affichage du graphique
    plt.show()"""


def tracer_arc_de_cercle(centre, rayon, point1, point2):
    # Calcul des angles entre les deux points et l'horizontale
    angle1 = angle(centre, point1)
    angle2 = angle(centre, point2)
    
    # Générer des angles intermédiaires entre les deux points
    if angle1 < angle2:
        angles = np.linspace(angle1, angle2, 100)  # 100 points pour tracer l'arc
    else:
        angles = np.linspace(angle1, angle2 + 2 * np.pi, 100)  # Pour traiter le cas où l'arc passe par 0 radians

    # Calculer les coordonnées des points de l'arc
    x_arc = centre[0] + rayon * np.cos(angles)
    y_arc = centre[1] + rayon * np.sin(angles)

    # Tracer l'arc de cercle
    plt.plot(x_arc, y_arc, 'b')

def tracegr(g,transpg=0.5,transpa=0.15,col="blue"):
    i=0
    fig, ax = plt.subplots(figsize = (10,10))
    color = 'blue'
    area = (20)**2
    x,y = [],[]
    for s in g :
        xs,ys=s
        x.append(xs)
        y.append(ys)
    #plt.scatter(x, y, c=color)#, s=area, alpha=transpg,linewidths=0.01)
    for a in g :
        i+=1
        for b in g[a]["adj"]:
            if len(g[a]["adj"][b])==1 :
                xa,ya=a
                xb,yb=b
                plt.plot(xa,ya, marker="x", color="blue")
                ax.plot([xa,xb], [ya,yb], color="blue")#, alpha = transpa,linewidth=0.01, markersize=0.01)
            else :
                (d,C) = g[a]["adj"][b]
                tracer_arc_de_cercle(C[0],C[1],a,b)

def ajoutept(g,a,liobs):
    sommet(g,a,a)
    for (c,r) in liobs :
        x,y=c
        gam=angle(c,a)
        tet=acos(r/dist(c,a))
        e=(x+r*cos(gam+tet-pi),y+r*sin(gam+tet-pi))
        f=(x+r*cos(pi+gam-tet),y+r*sin(gam+pi-tet))
        sommet(g,e,c)
        sommet(g,f,c)
        if a!=e and verif(a,e,(a,0),(c,r),liobs):
            arretedroite(g,a,e)
        if a!=f and verif(a,f,(a,0),(c,r),liobs):
            arretedroite(g,a,f)

        for s in g :
            d=g[s]
            
            if d["centre"]==c :
                #if c == (-44, -95):print(s)
                arretecourbe(g,s,e,(c,r))
                arretecourbe(g,s,f,(c,r))

            
        """
        if s!=a and verif(a,s,(a,0),g[s]["centre"],Liobs):
            arretedroite(g,a,s)"""

def tracechem(l):
    for i in range(len(l)-1):
        tracer_ligne(l[i],l[i+1],'r')

def path(g,l,pas=1):
    path=[]
    for i in range(len(l)-1):
        a=l[i]
        b=l[i+1]
        arrete=g[a]["adj"][b]
        d=arrete[0]
        #print("distance = ",arrete[0])
        n=int(d//pas)
        if n == 0:
            n=1
        pr=d/n
        #print("n = ",n)
        #print("pr = ",pr)
        if len(arrete)==1:
            
            for j in range(n):
                path.append((a[0]+j*pr*(b[0]-a[0])/d,a[1]+j*pr*(b[1]-a[1])/d))
        else :
            (centre,r)=arrete[1]
            r=dist(centre,a)

            angle1 = angle(a, centre)
            angle2 = angle(b, centre)
            # Générer des angles intermédiaires entre les deux points
            if abs(angle2-angle1) < pi  :
                angles = np.linspace(angle1, angle2, n)  # 100 points pour tracer l'arc
            else:
                angles = np.linspace(angle1,angle2+2*pi, n)  # Pour traiter le cas où l'arc passe par 0 radians
            
            # Calculer les coordonnées des points de l'arc
            x_arc = centre[0] + r * np.cos(angles)
            y_arc = centre[1] + r * np.sin(angles)

            for j in range(len(x_arc)-1):
                path.append((x_arc[j],y_arc[j]))
    return path





Liobs =[((120, -50), 35),((-152, -6), 55),((110, 135), 50),((12, -102), 30),((92, 170), 30),((-92, 176), 40),((-40, 220), 32),((-44, -95), 32),((-30, -150), 32)]
#Liobs =[((120, -50), 35),((-152, -6), 55),((110, 135), 50),((12, -102), 30),((92, 170), 30)]
#Liobs =[((120, -50), 35),((-152, -6), 55)]


def pathfinding(a,b,Liobs):
    llignes =[]
    g={}

    for i in range(len(Liobs)-1):
        for j in range(i+1,len(Liobs)):
            pointscotan(Liobs[i],Liobs[j],llignes,Liobs)

    for (a0,c1,b0,c2) in llignes:
        sommet(g,a0,c1[0])
        sommet(g,b0,c2[0])
        arretedroite(g,a0,b0)
        for s in g :
            d=g[s]
            #print(a)
            if d["centre"]==c1 and s!=a0:
                arretecourbe(g,s,a0,c1)
            if d["centre"]==c2 and s!=b0:
                arretecourbe(g,s,b0,c2)

    """
    a=(120,250)
    b=(-50,-250)
    c=(-75.54357875539259, -100.38540985462997)
    d=(-70.53320512217539, -77.11176291680574)"""

    ajoutept(g,a,Liobs)
    ajoutept(g,b,Liobs)

    l,p=diststar(g,b,a)
    path0=path(g,l)
    return(path0)

#print(pathfinding((-200,-200),(200,200),Liobs))

#print(l)
#print(path)
#tracegr(g)


#for (x,y) in path :
 #   plt.plot(x,y,color='r',marker='x')


"""
print(g[d])
for (x,y) in g[c]["adj"] :
    plt.plot(x,y,color='r',marker='x')"""

#print(path)
"""
for i in range(len(path)-1):
    print(dist(path[i],path[i+1]))"""



#plt.show()
