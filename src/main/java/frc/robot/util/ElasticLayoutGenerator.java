package frc.robot.util;

import java.io.FileWriter;
import java.io.IOException;
import javax.json.Json;
import javax.json.JsonArrayBuilder;
import javax.json.JsonObjectBuilder;

public class ElasticLayoutGenerator {
  public static final int[][] branchCoords = {
    {4, 4}, {6, 4}, {8, 4}, {10, 4}, {10, 1}, {8, 1}, {6, 1}, {4, 1}, {2, 1}, {0, 1}, {0, 4}, {2, 4}
  };

  public static void main(String args[]) throws IOException {
    JsonObjectBuilder coralBuilder = Json.createObjectBuilder();
    coralBuilder.add("name", "Coral");

    JsonArrayBuilder layoutsBuilder = Json.createArrayBuilder();
    for (int branch = 0; branch < 12; branch++) {
      JsonObjectBuilder branchBuilder = Json.createObjectBuilder();
      branchBuilder
          .add("title", "" + (char) ('A' + branch))
          .add("x", branchCoords[branch][0] * 64.0)
          .add("y", branchCoords[branch][1] * 64.0)
          .add("width", 128.0)
          .add("height", 192.0)
          .add("type", "List Layout")
          .add("properties", Json.createObjectBuilder().add("label_position", "HIDDEN"));

      JsonArrayBuilder childrenBuilder = Json.createArrayBuilder();
      for (int level = 4; level >= 2; level--) {
        childrenBuilder.add(
            Json.createObjectBuilder()
                .add("title", "Filled " + (char) ('A' + branch) + " L" + level)
                .add("x", 0.0)
                .add("y", 0.0)
                .add("width", 128.0)
                .add("height", 128.0)
                .add("type", "Toggle Switch")
                .add(
                    "properties",
                    Json.createObjectBuilder()
                        .add(
                            "topic",
                            "/SmartDashboard/" + "Filled " + (char) ('A' + branch) + " L" + level)
                        .add("period", 0.06)
                        .add("data_type", "boolean")));
      }

      branchBuilder.add("children", childrenBuilder);
      layoutsBuilder.add(branchBuilder);
    }

    coralBuilder.add(
        "grid_layout",
        Json.createObjectBuilder()
            .add("layouts", layoutsBuilder)
            .add("containers", Json.createArrayBuilder()));

    try (FileWriter file = new FileWriter("ElasticLayoutGeneratorOutput.json")) {
      file.write(coralBuilder.build().toString());
      System.out.println("File successfully written!");
    }
  }
}
