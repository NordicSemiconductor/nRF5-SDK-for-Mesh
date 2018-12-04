# Zephyr RTOS binaries

If you want to evaluate the Low Power feature, the Low Power node needs a Friend node in its mesh
network. Because the nRF5 SDK for Mesh does not currently support the Friend feature, you can use the pre-compiled
binaries running the @link_zephyr stack with Bluetooth mesh Friend support. Choose the binary corresponding to your
hardware platform.

The binaries are compiled from `v1.13.0` (`d62e861a01`) using the
<a href="https://github.com/zephyrproject-rtos/zephyr/tree/zephyr-v1.13.0/samples/bluetooth/mesh" target="_blank">
Bluetooth Mesh example</a> with the following changes to the `prj.conf`:

```diff
diff --git a/samples/bluetooth/mesh/prj.conf b/samples/bluetooth/mesh/prj.conf
index a8a0f5f37e..180e7062d3 100644
--- a/samples/bluetooth/mesh/prj.conf
+++ b/samples/bluetooth/mesh/prj.conf
@@ -52,21 +52,22 @@ CONFIG_BT_MESH_LABEL_COUNT=3

 #CONFIG_BT_MESH_IV_UPDATE_TEST=y

-#CONFIG_UART_CONSOLE=n
+CONFIG_RTT_CONSOLE=y
+CONFIG_UART_CONSOLE=n
 #CONFIG_BT_DEBUG_MONITOR=y
 CONFIG_BT_DEBUG_LOG=y

 CONFIG_BT_MESH_DEBUG=y
-#CONFIG_BT_MESH_DEBUG_PROV=y
-#CONFIG_BT_MESH_DEBUG_PROXY=y
-#CONFIG_BT_MESH_DEBUG_BEACON=y
+CONFIG_BT_MESH_DEBUG_PROV=n
+CONFIG_BT_MESH_DEBUG_PROXY=n
+CONFIG_BT_MESH_DEBUG_BEACON=n
 CONFIG_BT_MESH_DEBUG_NET=y
 CONFIG_BT_MESH_DEBUG_TRANS=y
-CONFIG_BT_MESH_DEBUG_SETTINGS=y
-#CONFIG_BT_MESH_DEBUG_LOW_POWER=y
+CONFIG_BT_MESH_DEBUG_SETTINGS=n
+CONFIG_BT_MESH_DEBUG_LOW_POWER=n
 CONFIG_BT_MESH_DEBUG_FRIEND=y
-#CONFIG_BT_MESH_DEBUG_MODEL=y
-#CONFIG_BT_MESH_DEBUG_ACCESS=y
-#CONFIG_BT_MESH_DEBUG_CRYPTO=y
-#CONFIG_BT_MESH_DEBUG_ADV=y
+CONFIG_BT_MESH_DEBUG_MODEL=y
+CONFIG_BT_MESH_DEBUG_ACCESS=y
+CONFIG_BT_MESH_DEBUG_CRYPTO=y
+CONFIG_BT_MESH_DEBUG_ADV=n
 #CONFIG_BT_MESH_SELF_TEST=y
diff --git a/samples/bluetooth/mesh/src/main.c b/samples/bluetooth/mesh/src/main.c
index 86ebe2300a..eba9759bce 100644
--- a/samples/bluetooth/mesh/src/main.c
+++ b/samples/bluetooth/mesh/src/main.c
@@ -40,94 +40,9 @@ static struct bt_mesh_health_srv health_srv = {

 BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

-static struct bt_mesh_model_pub gen_level_pub;
-static struct bt_mesh_model_pub gen_onoff_pub;
-
-static void gen_onoff_get(struct bt_mesh_model *model,
-			  struct bt_mesh_msg_ctx *ctx,
-			  struct net_buf_simple *buf)
-{
-}
-
-static void gen_onoff_set(struct bt_mesh_model *model,
-			  struct bt_mesh_msg_ctx *ctx,
-			  struct net_buf_simple *buf)
-{
-}
-
-static void gen_onoff_set_unack(struct bt_mesh_model *model,
-				struct bt_mesh_msg_ctx *ctx,
-				struct net_buf_simple *buf)
-{
-}
-
-static const struct bt_mesh_model_op gen_onoff_op[] = {
-	{ BT_MESH_MODEL_OP_2(0x82, 0x01), 0, gen_onoff_get },
-	{ BT_MESH_MODEL_OP_2(0x82, 0x02), 2, gen_onoff_set },
-	{ BT_MESH_MODEL_OP_2(0x82, 0x03), 2, gen_onoff_set_unack },
-	BT_MESH_MODEL_OP_END,
-};
-
-static void gen_level_get(struct bt_mesh_model *model,
-			  struct bt_mesh_msg_ctx *ctx,
-			  struct net_buf_simple *buf)
-{
-}
-
-static void gen_level_set(struct bt_mesh_model *model,
-			  struct bt_mesh_msg_ctx *ctx,
-			  struct net_buf_simple *buf)
-{
-}
-
-static void gen_level_set_unack(struct bt_mesh_model *model,
-				struct bt_mesh_msg_ctx *ctx,
-				struct net_buf_simple *buf)
-{
-}
-
-static void gen_delta_set(struct bt_mesh_model *model,
-			  struct bt_mesh_msg_ctx *ctx,
-			  struct net_buf_simple *buf)
-{
-}
-
-static void gen_delta_set_unack(struct bt_mesh_model *model,
-				struct bt_mesh_msg_ctx *ctx,
-				struct net_buf_simple *buf)
-{
-}
-
-static void gen_move_set(struct bt_mesh_model *model,
-			 struct bt_mesh_msg_ctx *ctx,
-			 struct net_buf_simple *buf)
-{
-}
-
-static void gen_move_set_unack(struct bt_mesh_model *model,
-			       struct bt_mesh_msg_ctx *ctx,
-			       struct net_buf_simple *buf)
-{
-}
-
-static const struct bt_mesh_model_op gen_level_op[] = {
-	{ BT_MESH_MODEL_OP_2(0x82, 0x05), 0, gen_level_get },
-	{ BT_MESH_MODEL_OP_2(0x82, 0x06), 3, gen_level_set },
-	{ BT_MESH_MODEL_OP_2(0x82, 0x07), 3, gen_level_set_unack },
-	{ BT_MESH_MODEL_OP_2(0x82, 0x09), 5, gen_delta_set },
-	{ BT_MESH_MODEL_OP_2(0x82, 0x0a), 5, gen_delta_set_unack },
-	{ BT_MESH_MODEL_OP_2(0x82, 0x0b), 3, gen_move_set },
-	{ BT_MESH_MODEL_OP_2(0x82, 0x0c), 3, gen_move_set_unack },
-	BT_MESH_MODEL_OP_END,
-};
-
 static struct bt_mesh_model root_models[] = {
 	BT_MESH_MODEL_CFG_SRV(&cfg_srv),
 	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
-	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_op,
-		      &gen_onoff_pub, NULL),
-	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_LEVEL_SRV, gen_level_op,
-		      &gen_level_pub, NULL),
 };

 static struct bt_mesh_elem elements[] = {
@@ -140,15 +55,6 @@ static const struct bt_mesh_comp comp = {
 	.elem_count = ARRAY_SIZE(elements),
 };

-static int output_number(bt_mesh_output_action_t action, u32_t number)
-{
-	printk("OOB Number: %u\n", number);
-
-	board_output_number(action, number);
-
-	return 0;
-}
-
 static void prov_complete(u16_t net_idx, u16_t addr)
 {
 	board_prov_complete();
@@ -163,9 +69,9 @@ static const uint8_t dev_uuid[16] = { 0xdd, 0xdd };

 static const struct bt_mesh_prov prov = {
 	.uuid = dev_uuid,
-	.output_size = 4,
-	.output_actions = BT_MESH_DISPLAY_NUMBER,
-	.output_number = output_number,
+	.output_size = 0,
+	.output_actions = BT_MESH_NO_OUTPUT,
+	.output_number = NULL,
 	.complete = prov_complete,
 	.reset = prov_reset,
 };
```

## Licensing

The Zephyr RTOS is licensed under
<a href="https://github.com/zephyrproject-rtos/zephyr/blob/master/LICENSE" target="_blank">Apache 2.0</a>.
The project also imports or reuses other packages of which licenses are described under
<a href="https://docs.zephyrproject.org/latest/LICENSING.html" target="_blank">
Licensing of Zephyr Project components</a>. A copy of the Zephyr RTOS Apache 2.0 license
is provided in `ZEPHYR_LICENSE`.
