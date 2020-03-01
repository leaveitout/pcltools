# Point Cloud Processing

This library can be used as the basis of building multi-threaded applications for the processing of point clouds. It uses native threads in C++11, with threads associated with individaul nodes in a processing graph to ensure native speed. Users of the library easily implement their own buffers and nodes and enjoy substantial time savings in processing-intensive pipelines. 

## Getting Started

If you are using PCL already, you should be good to fork and include this libary as a submodule in your project, otherwise please follow the following prerequisites.

### Prerequisites

A version of the Point Cloud Library will need to be installed (>1.7), as well as a C++11 enabled compiler (gcc > 5.0). Installation of PCL should be performed [as per the repo instructions](https://github.com/PointCloudLibrary/pcl).

### Installing

After forking the repo, it can be added as a submodule to a project and compiled as a library by adding the necessary steps to the `cmakelists.txt`.

## Example Use Case

An example use case is if you wanted to detect a table, segment all points above the table and then use these points for clustering. Using the standard PCL approach would involve most parts of the pipeline using a single thread. This would be made more challenging if you wanted to do this for multiple cameras simultaneously. To speed up processing by taking advantage of all of the unused threads on a user's machine, the following pipeline could be constructed with this library. 

![Example processing pipeline.](https://raw.githubusercontent.com/leaveitout/pcltools/master/res/pcl_software_pipeline_landscape.png)

## Citation

Please cite the following publication, if you make use of this library.

```
@inproceedings{Bruton2019RecognisingAF,
  title={Recognising Actions for Instructional Training using Pose Information: A Comparative Evaluation},
  author={Se{\'a}n Bruton and Gerard Lacey},
  booktitle={VISAPP},
  year={2019}
}
```


## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

This code was developed while the author was funded by an Irish Research Council PhD Scholarship.

