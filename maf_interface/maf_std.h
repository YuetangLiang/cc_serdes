#pragma once
#include <stdint.h>
#include <string>
#include <vector>

#define serdes1(a)    \
    template <class B>   \
    void serialize(B& buf) const { buf << (a);}\
    template <class B>   \
    void parse(B& buf) { buf >> (a);}

#define serdes2(a,b)    \
    template <class B>   \
    void serialize(B& buf) const { buf << (a) << (b);}\
    template <class B>   \
    void parse(B& buf) { buf >> (a) >> (b);}

#define serdes3(a,b,c)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c);}

#define serdes4(a,b,c,d)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d);}

#define serdes5(a,b,c,d,e)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e);}


#define serdes6(a,b,c,d,e,f)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f);}

#define serdes7(a,b,c,d,e,f,g)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g);}

#define serdes8(a,b,c,d,e,f,g,h)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h);}


#define serdes9(a,b,c,d,e,f,g,h,i)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i);}


#define serdes10(a,b,c,d,e,f,g,h,i,j)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j);}


#define serdes11(a,b,c,d,e,f,g,h,i,j,k)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k);}


#define serdes12(a,b,c,d,e,f,g,h,i,j,k,l)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l);}

#define serdes13(a,b,c,d,e,f,g,h,i,j,k,l,m)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m);}

#define serdes14(a,b,c,d,e,f,g,h,i,j,k,l,m,n)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n);}


#define serdes15(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o);}


#define serdes16(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p);}


#define serdes17(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p) << (q);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p) >> (q);}


#define serdes18(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p) << (q) << (r);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p) >> (q) >> (r);}

#define serdes19(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p) << (q) << (r) << (s);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p) >> (q) >> (r) >> (s);}



#define serdes20(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p) << (q) << (r) << (s) << (t);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p) >> (q) >> (r) >> (s) >> (t);}


#define serdes21(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p) << (q) << (r) << (s) << (t) << (u);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p) >> (q) >> (r) >> (s) >> (t) >> (u);}


#define serdes22(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p) << (q) << (r) << (s) << (t) << (u) << (v);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p) >> (q) >> (r) >> (s) >> (t) >> (u) >> (v);}


#define serdes23(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p) << (q) << (r) << (s) << (t) << (u) << (v) << (w);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p) >> (q) >> (r) >> (s) >> (t) >> (u) >> (v) >> (w);}


#define serdes24(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p) << (q) << (r) << (s) << (t) << (u) << (v) << (w) << (x);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p) >> (q) >> (r) >> (s) >> (t) >> (u) >> (v) >> (w) >> (x);}


#define serdes25(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,y)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p) << (q) << (r) << (s) << (t) << (u) << (v) << (w) << (x) << (y);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p) >> (q) >> (r) >> (s) >> (t) >> (u) >> (v) >> (w) >> (x) >> (y);}


#define serdes26(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,y,z)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p) << (q) << (r) << (s) << (t) << (u) << (v) << (w) << (x) << (y) << (z);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p) >> (q) >> (r) >> (s) >> (t) >> (u) >> (v) >> (w) >> (x) >> (y) >> (z);}


#define serdes27(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,y,z,a27)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p) << (q) << (r) << (s) << (t) << (u) << (v) << (w) << (x) << (y) << (z) << (a27);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p) >> (q) >> (r) >> (s) >> (t) >> (u) >> (v) >> (w) >> (x) >> (y) >> (z) >> (a27);}

#define serdes28(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,y,z,a27,a28)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p) << (q) << (r) << (s) << (t) << (u) << (v) << (w) << (x) << (y) << (z) << (a27) << (a28);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p) >> (q) >> (r) >> (s) >> (t) >> (u) >> (v) >> (w) >> (x) >> (y) >> (z) >> (a27) >> (a28);}

#define serdes29(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,y,z,a27,a28,a29)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p) << (q) << (r) << (s) << (t) << (u) << (v) << (w) << (x) << (y) << (z) << (a27) << (a28) << (a29);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p) >> (q) >> (r) >> (s) >> (t) >> (u) >> (v) >> (w) >> (x) >> (y) >> (z) >> (a27) >> (a28) >> (a29);}


#define serdes30(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,y,z,a27,a28,a29,a30)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p) << (q) << (r) << (s) << (t) << (u) << (v) << (w) << (x) << (y) << (z) << (a27) << (a28) << (a29) << (a30);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p) >> (q) >> (r) >> (s) >> (t) >> (u) >> (v) >> (w) >> (x) >> (y) >> (z) >> (a27) >> (a28) >> (a29) >> (a30);}


#define serdes31(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,y,z,a27,a28,a29,a30,a31)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p) << (q) << (r) << (s) << (t) << (u) << (v) << (w) << (x) << (y) << (z) << (a27) << (a28) << (a29) << (a30) << (a31);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p) >> (q) >> (r) >> (s) >> (t) >> (u) >> (v) >> (w) >> (x) >> (y) >> (z) >> (a27) >> (a28) >> (a29) >> (a30) >> (a31);}


#define serdes32(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,y,z,a27,a28,a29,a30,a31,a32)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p) << (q) << (r) << (s) << (t) << (u) << (v) << (w) << (x) << (y) << (z) << (a27) << (a28) << (a29) << (a30) << (a31) << (a32);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p) >> (q) >> (r) >> (s) >> (t) >> (u) >> (v) >> (w) >> (x) >> (y) >> (z) >> (a27) >> (a28) >> (a29) >> (a30) >> (a31) >> (a32);}


#define serdes33(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,y,z,a27,a28,a29,a30,a31,a32,a33)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p) << (q) << (r) << (s) << (t) << (u) << (v) << (w) << (x) << (y) << (z) << (a27) << (a28) << (a29) << (a30) << (a31) << (a32) << (a33);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p) >> (q) >> (r) >> (s) >> (t) >> (u) >> (v) >> (w) >> (x) >> (y) >> (z) >> (a27) >> (a28) >> (a29) >> (a30) >> (a31) >> (a32) >> (a33);}

#define serdes34(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,y,z,a27,a28,a29,a30,a31,a32,a33,a34)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p) << (q) << (r) << (s) << (t) << (u) << (v) << (w) << (x) << (y) << (z) << (a27) << (a28) << (a29) << (a30) << (a31) << (a32) << (a33) << (a34);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p) >> (q) >> (r) >> (s) >> (t) >> (u) >> (v) >> (w) >> (x) >> (y) >> (z) >> (a27) >> (a28) >> (a29) >> (a30) >> (a31) >> (a32) >> (a33) >> (a34);}

#define serdes35(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,y,z,a27,a28,a29,a30,a31,a32,a33,a34,a35)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p) << (q) << (r) << (s) << (t) << (u) << (v) << (w) << (x) << (y) << (z) << (a27) << (a28) << (a29) << (a30) << (a31) << (a32) << (a33) << (a34) << (a35);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p) >> (q) >> (r) >> (s) >> (t) >> (u) >> (v) >> (w) >> (x) >> (y) >> (z) >> (a27) >> (a28) >> (a29) >> (a30) >> (a31) >> (a32) >> (a33) >> (a34) >> (a35);}


#define serdes36(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,y,z,a27,a28,a29,a30,a31,a32,a33,a34,a35,a36)\
    template <class B> void serialize(B& buf) const { buf << (a) << (b) << (c) << (d) << (e) << (f) << (g) << (h) << (i) << (j) << (k) << (l) << (m) << (n) << (o) << (p) << (q) << (r) << (s) << (t) << (u) << (v) << (w) << (x) << (y) << (z) << (a27) << (a28) << (a29) << (a30) << (a31) << (a32) << (a33) << (a34) << (a35) << (a36);} \
    template <class B> void parse(B& buf) { buf >> (a) >> (b) >> (c) >> (d) >> (e) >> (f) >> (g) >> (h) >> (i) >> (j) >> (k) >> (l) >> (m) >> (n) >> (o) >> (p) >> (q) >> (r) >> (s) >> (t) >> (u) >> (v) >> (w) >> (x) >> (y) >> (z) >> (a27) >> (a28) >> (a29) >> (a30) >> (a31) >> (a32) >> (a33) >> (a34) >> (a35) >> (a36);}


namespace maf_std {

struct UInt64 {
  uint64_t data;
};

struct Header {
  uint32_t seq;
  uint64_t stamp;
  std::string frame_id;
    serdes3(seq,stamp,frame_id)
};

struct UInt32 {
  uint32_t data;
};

struct Int8 {
  int8_t data;
};

struct Duration {
  int64_t data;
};

struct UInt8 {
  uint8_t data;
};

struct MultiArrayDimension {
  std::string label;
  uint32_t size;
  uint32_t stride;
};

struct UInt16 {
  uint16_t data;
};

struct Char {
  uint8_t data;
};

struct Int16 {
  int16_t data;
};

struct Float32 {
  float data;
};

struct Byte {
  int8_t data;
};

struct Time {
  uint64_t data;
};

struct Int64 {
  int64_t data;
};

struct Empty {};

struct Float64 {
  double data;
};

struct Int32 {
  int32_t data;
};

struct String {
  std::string data;
};

struct ColorRGBA {
  float r;
  float g;
  float b;
  float a;
};

struct Bool {
  bool data;
};

struct MultiArrayLayout {
  std::vector<MultiArrayDimension> dim;
  uint32_t data_offset;
};

struct UInt16MultiArray {
  MultiArrayLayout layout;
  std::vector<uint16_t> data;
};

struct UInt64MultiArray {
  MultiArrayLayout layout;
  std::vector<uint64_t> data;
};

struct Float32MultiArray {
  MultiArrayLayout layout;
  std::vector<float> data;
};

struct Float64MultiArray {
  MultiArrayLayout layout;
  std::vector<double> data;
};

struct Int32MultiArray {
  MultiArrayLayout layout;
  std::vector<int32_t> data;
};

struct Int64MultiArray {
  MultiArrayLayout layout;
  std::vector<int64_t> data;
};

struct Int16MultiArray {
  MultiArrayLayout layout;
  std::vector<int16_t> data;
};

struct UInt32MultiArray {
  MultiArrayLayout layout;
  std::vector<uint32_t> data;
};

struct Int8MultiArray {
  MultiArrayLayout layout;
  std::vector<int8_t> data;
};

struct UInt8MultiArray {
  MultiArrayLayout layout;
  std::vector<uint8_t> data;
};

struct ByteMultiArray {
  MultiArrayLayout layout;
  std::vector<int8_t> data;
};

} // namespace maf_std
