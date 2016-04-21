CURR_DIR=$(dirname "$0")
UE_DIR=$CURR_DIR/../UnrealEngine

if [ -d $UE_DIR ]
then
    echo "Found UnrealEngine directory:" $UE_DIR
else
    echo "Error: UnrealEngine is expected to be at:" $UE_DIR
    exit 1
fi

cd $UE_DIR/Engine/Build/BatchFiles/Mac
sh GenerateProjectFiles.sh -project=$CURR_DIR/ClearPath.uproject -game