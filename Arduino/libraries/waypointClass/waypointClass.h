class  waypointClass
{
    
  public:
    waypointClass(float pLat = 0, float pLong = 0)
      {
        fLat = pLat;
        fLong = pLong;
      }
      
    float getLat(void) {return fLat;}
    float getLong(void) {return fLong;}

  private:
    float fLat, fLong;
      
  
};  // waypointClass


// usage as array: 
//waypointClass myWaypoints[4] = {waypointClass(1,2), waypointClass(2,3), waypointClass(4,5), waypointClass() };


