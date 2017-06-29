class GenerateData
{ 
  public
	static getData()
	{

		char *data_ptr;
  	long unsigned int initial_address =  (long unsigned int)&(*data_ptr);
  
		for(int i =0 ;i < 1000; i++)
  		{
    
    		int a = 20;
    		memcpy(data_ptr,&a,sizeof(a));
    		data_ptr += sizeof(a);
    	}
   	
    long unsigned int final_address =  (long unsigned int)&(*data_ptr);
   	size_t mes_size = (final_address-initial_address);
   	printf("%d\n", mes_size);  
  }
}