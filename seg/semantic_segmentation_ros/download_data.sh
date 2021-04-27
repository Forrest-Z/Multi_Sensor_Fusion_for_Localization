# 下载MIT20K数据集
wget -O ./data/ADEChallengeData2016.zip http://data.csail.mit.edu/places/ADEchallenge/ADEChallengeData2016.zip
unzip ./data/ADEChallengeData2016.zip -d ./data
rm ./data/ADEChallengeData2016.zip
echo "Dataset downloaded."

# 下载预训练模型
放到semantic_segmantation_ros/ckpt/文件夹下面
http://sceneparsing.csail.mit.edu/model/pytorch/
