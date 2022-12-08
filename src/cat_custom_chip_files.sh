cat hashmap.h ow.h ow_signaling_sm.c ds18b20-2.chip.c hashmap.c  | grep -Ev "include.*(ow|hashmap)"> ds18b20-all.chip.c
echo "// ========== `date` ====== " >> ds18b20-all.chip.c