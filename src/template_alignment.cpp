#include <chain_link_alignment.h>

// Align a collection of object templates to a sample point cloud
int
main (int argc, char **argv)
{
  //int alignN = 1;
    bool alignAll = false;
    if (argc < 2)
    {
        printf ("No templates file given!\n");
        printf ("Use ./template_alignment <templates.txt> <target.pcd> [-a]\n");
        return (-1);
    }
    else if (argc < 3)
    {
        printf ("No target PCD file given!\n");
        printf ("Use ./template_alignment <templates.txt> <target.pcd> [-a]\n");
        return (-1);
    }
    else if (argc >= 4)
    {
        alignAll = true;
        /*try
        {
            alignN = atoi(argv[3]);
        }
        catch (std::exception)
        {
            printf ("The third parameter should be the number of templates to return\n");
        }*/
    }
    else { }

    TemplateAligner aligner = TemplateAligner(argv[1], argv[2]);
    //if (alignN == 1)
    if (!alignAll)
    {
        aligner.Align();
    }
    else
    {
        aligner.AlignAll();
    }

    return (0);
}

