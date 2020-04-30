#ifndef vicalib_h
#define vicalib_h

#ifdef __cplusplus
extern "C" {
#endif
 
int vicalib(int argc, char *argv[], unsigned char * json_data, int json_data_length, double reprojection_errors[2]);

#ifdef __cplusplus
}
#endif

#endif //vicalib_h