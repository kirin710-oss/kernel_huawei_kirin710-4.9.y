/*
 * Copyright (C) Huawei Technologies Co., Ltd. 2012-2015. All rights reserved.
 * foss@huawei.com
 *
 * If distributed as part of the Linux kernel, the following license terms
 * apply:
 *
 * * This program is free software; you can redistribute it and/or modify
 * * it under the terms of the GNU General Public License version 2 and
 * * only version 2 as published by the Free Software Foundation.
 * *
 * * This program is distributed in the hope that it will be useful,
 * * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * * GNU General Public License for more details.
 * *
 * * You should have received a copy of the GNU General Public License
 * * along with this program; if not, write to the Free Software
 * * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 *
 * Otherwise, the following license terms apply:
 *
 * * Redistribution and use in source and binary forms, with or without
 * * modification, are permitted provided that the following conditions
 * * are met:
 * * 1) Redistributions of source code must retain the above copyright
 * *    notice, this list of conditions and the following disclaimer.
 * * 2) Redistributions in binary form must reproduce the above copyright
 * *    notice, this list of conditions and the following disclaimer in the
 * *    documentation and/or other materials provided with the distribution.
 * * 3) Neither the name of Huawei nor the names of its contributors may
 * *    be used to endorse or promote products derived from this software
 * *    without specific prior written permission.
 *
 * * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "product_config.h"
#include <linux/version.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/zlib.h>
#include <linux/dma-mapping.h>
#include <asm/dma-mapping.h>
#include <asm/cacheflush.h>
#include <soc_onchiprom.h>
#include <bsp_shared_ddr.h>
#include <bsp_reset.h>
#include <bsp_sec.h>
#include <bsp_rfile.h>
#include <bsp_version.h>
#include <bsp_ddr.h>
#include <bsp_efuse.h>
#include <bsp_print.h>
#include <bsp_nvim.h>
#define THIS_MODU mod_loadm
#include "load_image.h"
#include "modem_dtb.h"
#include "product_config.h"
#include "securec.h"

/* DallsÖ®ºóÊÖ»úºÍMBBÈÚºÏŽúÂë */

#define SECBOOT_BUFLEN  (0x100000)      /*1MB*/

#define MODEM_IMAGE_PATH    "/modem_fw/"
#define NVM_IMAGE_PATH      "/mnvm2:0/modem_nv/"
#define MBN_IMAGE_PATH      "/mnvm2:0/mbn_nv/"
#define NVM_IMAGE_PATH_DATA      "/mnt/modem/mnvm2:0/modem_nv/"
#define MBN_IMAGE_PATH_DATA      "/mnt/modem/mnvm2:0/mbn_nv/"
#define MODEM_COLD_PATCH_PATH "/patch_hw/modem_fw/"
#define VRL_SIZE                    (0x1000)  /*VRL 4K*/

#define MODEM_IMAGE_PATH_VENDOR    "/vendor/modem/modem_fw/"
#define MODEM_CERT_PATH             "/product/region_comm/china/modemCert/"




/* Žø°²È«OSÐèÒª°²È«ŒÓÔØ£¬Ô€ÁôÁ¬ÐøÄÚŽæ£¬·ñÔòÔÚÏµÍ³³€Ê±ŒäÔËÐÐºó£¬µ¥¶ÀžŽÎ»Ê±¿ÉÄÜÉêÇë²»µœÁ¬ÐøÄÚŽæ */
static  u8 *SECBOOT_BUFFER = NULL;

struct image_type_name
{
    enum SVC_SECBOOT_IMG_TYPE etype;
    u32 run_addr;
    u32 ddr_size;
    const char * dir;
    const char* name;
};

struct image_type_name  modem_images_root[] =
{
    {MODEM, DDR_MCORE_ADDR,         DDR_MCORE_SIZE,       MODEM_IMAGE_PATH,  "balong_modem.bin"},
    {HIFI,  DDR_HIFI_ADDR,          DDR_HIFI_SIZE,        MODEM_IMAGE_PATH,  "hifi.img"},/* Ô€Áô */
    {DSP,   DDR_TLPHY_IMAGE_ADDR,   DDR_TLPHY_IMAGE_SIZE, MODEM_IMAGE_PATH,  "phy.bin"},
    {TAS,   0,                      0,                    MODEM_IMAGE_PATH,  "tas.bin"},
    {WAS,   0,                      0,                    MODEM_IMAGE_PATH,  "was.bin"},
    {CAS,   0,                      0,                    MODEM_IMAGE_PATH,  "cas.bin"}, /* Ô€Áô */
    {MODEM_DTB, DDR_MCORE_DTS_ADDR, DDR_MCORE_DTS_SIZE,   MODEM_IMAGE_PATH,  "modem_dt.img"},
    {MODEM_COLD_PATCH, DDR_MCORE_ADDR,         DDR_MCORE_SIZE,       MODEM_COLD_PATCH_PATH,  "balong_modem.bin.p"},
    {DSP_COLD_PATCH,   DDR_MCORE_ADDR,         DDR_MCORE_SIZE,       MODEM_COLD_PATCH_PATH,  "phy.bin.p"},
    {SOC_MAX,       0,              0,                    "",                ""},
};

struct image_type_name modem_images_vendor[] =
{
    {MODEM, DDR_MCORE_ADDR,         DDR_MCORE_SIZE,       MODEM_IMAGE_PATH_VENDOR,  "balong_modem.bin"},
    {HIFI,  DDR_HIFI_ADDR,          DDR_HIFI_SIZE,        MODEM_IMAGE_PATH_VENDOR,  "hifi.img"},/* Ô€Áô */
    {DSP,   DDR_TLPHY_IMAGE_ADDR,   DDR_TLPHY_IMAGE_SIZE, MODEM_IMAGE_PATH_VENDOR,  "phy.bin"},
    {TAS,   0,                      0,                    MODEM_IMAGE_PATH_VENDOR,  "tas.bin"},
    {WAS,   0,                      0,                    MODEM_IMAGE_PATH_VENDOR,  "was.bin"},
    {CAS,   0,                      0,                    MODEM_IMAGE_PATH_VENDOR,  "cas.bin"}, /* Ô€Áô */
    {MODEM_DTB, DDR_MCORE_DTS_ADDR, DDR_MCORE_DTS_SIZE,   MODEM_IMAGE_PATH_VENDOR,  "modem_dt.img"},
    {MODEM_COLD_PATCH, DDR_MCORE_ADDR,         DDR_MCORE_SIZE,       MODEM_COLD_PATCH_PATH,  "balong_modem.bin.p"},
    {DSP_COLD_PATCH,   DDR_MCORE_ADDR,         DDR_MCORE_SIZE,       MODEM_COLD_PATCH_PATH,  "phy.bin.p"},
    {SOC_MAX,       0,              0,                    "",                ""},
};

struct image_type_name* modem_images = modem_images_root;

unsigned int g_images_size[SOC_MAX] = {0};

/*lint -save -e651 -e708 -e570 -e64 -e785*/
static DEFINE_MUTEX(load_proc_lock);
/*lint -restore */

 int modem_dir_init(void)
{
    if(!bsp_access((s8*) MODEM_IMAGE_PATH_VENDOR,0))
    {
         modem_images = modem_images_vendor;
     }
     else if(!bsp_access((s8*) MODEM_IMAGE_PATH,0))
     {
          modem_images = modem_images_root;
     }
     else
     {
         sec_print_err("error: path /vendor/modem/modem_fw/  and path /modem_fw/ can't access, return\n" );
         return -EACCES;
     }

    sec_print_info("info: path %s   can access\n", modem_images->dir);
    return 0;
}

static int get_image(struct image_type_name** image, enum SVC_SECBOOT_IMG_TYPE etype,u32 run_addr, u32 ddr_size)
{
    unsigned int i;
    struct image_type_name* img;

    img = modem_images;
    for(i=0; i<(sizeof(modem_images_vendor)/sizeof(struct image_type_name)); i++)
    {
        if(img->etype == etype)
        {
            break;
        }
        img++;
    }
    if(i == (sizeof(modem_images_vendor)/sizeof(struct image_type_name)))
    {
        /* cov_verified_start */
        sec_print_err("can not find image of type id %d\n", etype);
        return -ENOENT;
        /* cov_verified_stop */
    }
    /*Èç¹ûÊÇtas wasŸµÏñµÄ»°Òª*/
    if(run_addr && ddr_size)
    {
        img->run_addr = run_addr ;
        img->ddr_size = ddr_size ;
    }
    *image = (struct image_type_name*)img;

    return 0;
}

static int get_file_size(const char *filename)
{
    struct rfile_stat_stru st;
    s32 ret;
    memset_s(&st, sizeof(st), 0x00, sizeof (st));
    ret = bsp_stat((s8*)filename, (void *)&st);
    if(ret)
    {
        /* cov_verified_start */
        sec_print_err("file bsp_stat error .\n");
        return ret;
        /* cov_verified_stop */
    }

    return (int)st.size;
}

static int get_file_name(char *file_name, const struct image_type_name *image, bool *is_sec)
{
    /* ³¢ÊÔÒÔsec_¿ªÍ·µÄ°²È«ŸµÏñ */
    *is_sec = true;
    file_name[0] = '\0';
    strncat_s(file_name, LOADM_FILE_NAME_LEN, image->dir, strnlen(image->dir, LOADM_FILE_NAME_LEN));
    strncat_s(file_name, LOADM_FILE_NAME_LEN, "sec_", strnlen("sec_", LOADM_FILE_NAME_LEN));
    strncat_s(file_name, LOADM_FILE_NAME_LEN, image->name, strnlen(image->name, LOADM_FILE_NAME_LEN));
    sec_print_info("loading %s  image\n", file_name);
    /*if(bsp_access((s8*) file_name, RFILE_RDONLY))
    {
        sec_print_info("file %s can't access, try unsec image\n", file_name);

        *is_sec = false;
        file_name[0] = '\0';
        strncat_s(file_name, LOADM_FILE_NAME_LEN, image->dir, strnlen(image->dir, LOADM_FILE_NAME_LEN));
        strncat_s(file_name, LOADM_FILE_NAME_LEN, image->name, strnlen(image->name, LOADM_FILE_NAME_LEN));

        if(bsp_access((s8*) file_name, RFILE_RDONLY))
        {
            sec_print_err("error: file %s can't access, return\n", file_name);
            return -EACCES;
        }
    }*/

    return 0;
}

static int read_file(const char *file_name, unsigned int offset,
        unsigned int length, char *buffer)
{
    struct file * fp;
    int retval;
    loff_t file_pos = offset;
    fp = filp_open(file_name, O_RDONLY, 0600);
    if ((!fp) || (IS_ERR(fp))) {
        /* cov_verified_start */
        retval = (int)PTR_ERR(fp);
        sec_print_err("filp_open(%s) failed, ret:%d", file_name, retval);
        return retval;
        /* cov_verified_stop */
    }


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0))
    retval = kernel_read(fp, buffer, (size_t)length, &file_pos);
#else
    retval = kernel_read(fp, file_pos, buffer, (unsigned long)length);
#endif
    if (retval != (int)length) {
        /* cov_verified_start */
        sec_print_err("kernel_read(%s) failed, retval %d, require len %u\n",
            file_name, retval, length);
        if (retval >= 0)
            retval = -EIO;
        /* cov_verified_stop */
    }
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0))
    if(fp->f_inode)
    {
        invalidate_mapping_pages(fp->f_inode->i_mapping, 0, -1);/*lint !e747 !e570*/
    }
#endif
    filp_close(fp, NULL);
    return retval;
}
int gzip_header_check(unsigned char* zbuf)
{
    if (zbuf[0] != 0x1f || zbuf[1] != 0x8b || zbuf[2] != 0x08) {
        return 0;
    } else {
        return 1;
    }
}



static TEEC_Session load_session;
static TEEC_Context load_context;

/*
 * Function name:TEEK_init.
 * Discription:Init the TEEC and get the context
 * Parameters:
 *    @ session: the bridge from unsec world to sec world.
 *    @ context: context.
 * return value:
 *    @ TEEC_SUCCESS-->success, others-->failed.
 */
static int TEEK_init(void)
{
    TEEC_Result result;
    TEEC_UUID svc_uuid = TEE_SERVICE_SECBOOT;
    TEEC_Operation operation = {0};
    const char* package_name = "sec_boot";
    u32 root_id = 0;

    result = TEEK_InitializeContext(
            NULL,
            &load_context);

    if(result != TEEC_SUCCESS) {
        /* cov_verified_start */
        sec_print_err("TEEK_InitializeContext failed, result %#x\n", result);
        goto error;
        /* cov_verified_stop */
    }

    operation.started = 1;
    operation.cancel_flag = 0;
    operation.paramTypes =
        (u32)TEEC_PARAM_TYPES(TEEC_NONE, TEEC_NONE, TEEC_MEMREF_TEMP_INPUT, TEEC_MEMREF_TEMP_INPUT);/*lint !e845*/

    operation.params[2].tmpref.buffer = (void *)(&root_id);
    operation.params[2].tmpref.size = sizeof(root_id);
    operation.params[3].tmpref.buffer = (void *)(package_name);
    operation.params[3].tmpref.size = strlen(package_name) + 1;
    result = TEEK_OpenSession(
            &load_context,
            &load_session,
            &svc_uuid,
            TEEC_LOGIN_IDENTIFY,
            NULL,
            &operation,
            NULL);

    if (result != TEEC_SUCCESS)
    {
        /* cov_verified_start */
        sec_print_err("TEEK_OpenSession failed, result %#x\n", result);
        TEEK_FinalizeContext(&load_context);
        /* cov_verified_stop */
    }

error:

    return (int)result;
}

static void TEEK_uninit(void)
{
    TEEK_CloseSession(&load_session);
    TEEK_FinalizeContext(&load_context);
}


/*
 * Function name:trans_vrl_to_os.
 * Discription:transfer vrl data to sec_OS
 * Parameters:
 *    @ session: the bridge from unsec world to sec world.
 *    @ image: the data of the image to transfer.
 *    @ buf: the buf in  kernel to transfer
 *    @ size: the size to transfer.
 * return value:
 *    @ TEEC_SUCCESS-->success, others--> failed.
 */
static int trans_vrl_to_os(enum SVC_SECBOOT_IMG_TYPE  image,
          void * buf,
          const unsigned int size)
{
    TEEC_Session *session = &load_session;
    TEEC_Result result;
    TEEC_Operation operation;
    u32 origin;

    operation.started = 1;
    operation.cancel_flag = 0;
    operation.paramTypes =
        (u32)TEEC_PARAM_TYPES(TEEC_VALUE_INPUT, TEEC_MEMREF_TEMP_INPUT, TEEC_NONE, TEEC_NONE);/*lint !e845*/

    operation.params[0].value.a = image;
    operation.params[1].tmpref.buffer = (void *)buf;
    operation.params[1].tmpref.size = size;

    result = TEEK_InvokeCommand(
                session,
                SECBOOT_CMD_ID_COPY_VRL_TYPE,
                &operation,
                &origin);
    if (result != TEEC_SUCCESS) {
        /* cov_verified_start */
        sec_print_err("invoke failed, result %#x\n", result);
        /* cov_verified_stop */
    }

    return (int)result;
}

/*
 * Function name:trans_data_to_os.
 * Discription:transfer image data to sec_OS
 * Parameters:
 *    @ session: the bridge from unsec world to sec world.
 *    @ image: the data of the image to transfer.
 *    @ run_addr: the image entry address.
 *    @ buf: the buf in  kernel to transfer
 *    @ offset: the offset to run_addr.
 *    @ size: the size to transfer.
 * return value:
 *    @ TEEC_SUCCESS-->success, others--> failed.
 */
static int trans_data_to_os(enum SVC_SECBOOT_IMG_TYPE  image,
                        u32 run_addr,
                        void * buf,
                        const unsigned int offset,
                        const unsigned int size)
{
    TEEC_Session *session = &load_session;
    TEEC_Result result;
    TEEC_Operation operation;
    u32 origin;
    unsigned long paddr;
    paddr = MDDR_FAMA(run_addr);

    operation.started = 1;
    operation.cancel_flag = 0;
    operation.paramTypes =
        (u32)TEEC_PARAM_TYPES(TEEC_VALUE_INPUT, TEEC_VALUE_INPUT, TEEC_VALUE_INPUT, TEEC_VALUE_INPUT);/*lint !e845*/

    operation.params[0].value.a = image;
    operation.params[0].value.b = (u32)(paddr & 0xFFFFFFFF);;
    operation.params[1].value.a = (u32)((u64)paddr >> 32);/* ÊÖ»úºÍMBB ŒæÈÝ */
    operation.params[1].value.b = offset;
    operation.params[2].value.a = (u32)virt_to_phys(buf);/* ÊÖ»úºÍMBB ŒæÈÝ */
    operation.params[2].value.b = (u64)virt_to_phys(buf) >> 32;/* ÊÖ»úºÍMBB ŒæÈÝ */
    operation.params[3].value.a = size;
    result = TEEK_InvokeCommand(
                session,
                SECBOOT_CMD_ID_COPY_DATA_TYPE,
                &operation,
                &origin);
    if (result != TEEC_SUCCESS) {
        /* cov_verified_start */
        sec_print_err("invoke failed, result %#x\n", result);
        /* cov_verified_stop */
    }

    return (int)result;
}

/*
 * Function name:start_soc_image.
 * Discription:start the image verification, if success, unreset the soc
 * Parameters:
 *    @ session: the bridge from unsec world to sec world.
 *    @ image: the image to verification and unreset
 *    @ run_addr: the image entry address
 * return value:
 *    @ TEEC_SUCCESS-->success, others-->failed.
 */
static int verify_soc_image(enum SVC_SECBOOT_IMG_TYPE  image,
                        u32 run_addr)
{
    TEEC_Session *session = &load_session;
    TEEC_Result result;
    TEEC_Operation operation;
    u32 origin;
    unsigned long paddr;
    int ret;
    paddr = MDDR_FAMA(run_addr);

    ret = bsp_efuse_write_prepare();
    if(ret)
    {
        return ret;
    }
    operation.started = 1;
    operation.cancel_flag = 0;
    operation.paramTypes =
        (u32)TEEC_PARAM_TYPES(TEEC_VALUE_INPUT, TEEC_VALUE_INPUT, TEEC_NONE, TEEC_NONE);/*lint !e845*/

     operation.params[0].value.a = image;
     operation.params[0].value.b = 0;/*SECBOOT_LOCKSTATE , not used currently*/
     operation.params[1].value.a = (u32)(paddr & 0xFFFFFFFF);
     operation.params[1].value.b = (u32)((u64)paddr >> 32);/* ÊÖ»úºÍMBB ŒæÈÝ */
     result = TEEK_InvokeCommand(session,
                                   SECBOOT_CMD_ID_VERIFY_DATA_TYPE,
                                    &operation,
                                    &origin);
     if (result != TEEC_SUCCESS){
        /* cov_verified_start */
        sec_print_err("start  failed, result is 0x%x!\n", result);
        /* cov_verified_stop */
    }
    bsp_efuse_write_complete();
    return (int)result;
}


struct cold_patch_info_s g_cold_patch_info;
static u32 get_img_load_position_offset(const struct image_type_name* image,int img_size)
{
    if(!image)
    {
        sec_print_err("param image is NULL,please check!\n");
        return 0;
    }
    if((u32)img_size > image->ddr_size)
    {
        sec_print_err("param img_size is too large,soc_type = %d,img_size = 0x%x,ddr_size = 0x%x!\n",image->etype,img_size,image->ddr_size);
        return 0;
    }
    if(image->etype == MODEM)
    {
        /*Èç¹ûÒÑŸ­ŒÓÔØCCORE²¹¶¡ŸµÏñ£¬MODEM DDR¿ÕŒäµÄ×îºóÎ»ÖÃŽæ·ÅCCORE²¹¶¡ŸµÏñ£¬CCORE²¹¶¡ŸµÏñÖ®ÉÏŽæ·ÅÔ­CCOREŸµÏñ£¬ËùÒÔÔ­ŸµÏñµÄ
        ŒÓÔØÎ»ÖÃ=ddr_size - CCORE²¹¶¡ŸµÏñŽóÐ¡ - Ô­CCOREŸµÏñŽóÐ¡£»
        Èç¹ûCCORE²¹¶¡ŸµÏñ²»ŽæÔÚ»òŒÓÔØÊ§°Ü£¬Ô­CCOREŸµÏñŒÓÔØµœMODEM DDR¿ÕŒäµÄ×îºóÎ»ÖÃ*/
        if(g_cold_patch_info.modem_patch_info[CCORE_PATCH].patch_status == PUT_PATCH)
            return (u32)(image->ddr_size - g_images_size[MODEM_COLD_PATCH] -(u32)img_size);
        else
            return (u32)(image->ddr_size -(u32)img_size);
    }
    else if(image->etype == DSP)
    {
         /*Èç¹ûÒÑŸ­ŒÓÔØDSP²¹¶¡ŸµÏñ£¬ÓÉÓÚDSP DDR¿ÕŒäÓÐÏÞ£¬ËùÒÔœèÖúMODEM DDR¿ÕŒäŽæ·ÅDSP²¹¶¡ŸµÏñºÍÔ­DSPŸµÏñ£¬È»ºóœøÐÐŸµÏñÆŽœÓ£¬
         MODEM DDR¿ÕŒäµÄ×îºóÎ»ÖÃŽæ·ÅDSP²¹¶¡ŸµÏñ£¬DSP²¹¶¡ŸµÏñÖ®ÉÏŽæ·ÅÔ­DSPŸµÏñ£¬Ô­DSPŸµÏñŒÓÔØÎ»ÖÃ = DDR_MCORE_SIZE -
         DSP²¹¶¡ŸµÏñŽóÐ¡ - Ô­DSPŸµÏñŽóÐ¡£»
         Èç¹ûDSP²¹¶¡ŸµÏñ²»ŽæÔÚ»òŒÓÔØÊ§°Ü£¬Ô­DSPŸµÏñŒÓÔØµœDSP DDR¿ÕŒäÆ«ÒÆÎª0µÄµØÖ·*/
        if(g_cold_patch_info.modem_patch_info[DSP_PATCH].patch_status == PUT_PATCH)
            return (u32)(DDR_MCORE_SIZE - g_images_size[DSP_COLD_PATCH] -(u32)img_size);
        else
            return 0;
    }
    return 0;
}

/******************************************************************************
Function:       load_data_to_secos
Description:    ŽÓÖž¶šÆ«ÒÆ¿ªÊŒŽ«ËÍÖž¶šŽóÐ¡µÄŸµÏñ
Input:
            part_name   - Òª·¢ËÍŸµÏñµÄÃû³Æ
            offset    - Æ«ÒÆµØÖ·
            sizeToRead  - ÊäÈë²ÎÊý£¬ÒªÐŽÈëµÄŸµÏñµÄbytesŽóÐ¡

Output:         none
Return:         SEC_OK: OK  SEC_ERROR: ERRORÂë
******************************************************************************/
static int load_data_to_secos(const char* file_name, u32 offset, u32 size,
            const struct image_type_name* image, bool is_sec)
{
    int ret;
    int read_bytes;
    int readed_bytes;
    int remain_bytes;
    u32 file_offset = 0;
    u32 skip_offset = 0;
    u32 load_position_offset = 0;
    /* ¶ÁÈ¡Öž¶šÆ«ÒÆµÄÖž¶šŽóÐ¡ */
    if(0 != offset)
    {
        skip_offset = offset;
        remain_bytes = (int)size;
    }
    else    /* ¶ÁÈ¡ÕûžöÎÄŒþ */
    {
        remain_bytes = get_file_size(file_name);
        if (remain_bytes <=0)
        {
            sec_print_err("error file_size 0x%x\n", remain_bytes);
            return remain_bytes;
        }

        if(is_sec)
        {
            if(remain_bytes <= VRL_SIZE)
            {
                sec_print_err("error file_size (0x%x) less than VRL_SIZE\n", remain_bytes);
                return -EIO;
            }
            remain_bytes -= VRL_SIZE;
            skip_offset = VRL_SIZE;
        }
    }

    /* Œì²é¶ÁÈ¡µÄŽóÐ¡ÊÇ·ñ³¬¹ýddr·ÖÇøŽóÐ¡ */
    if((u32)remain_bytes > image->ddr_size)
    {
        /* cov_verified_start */
        sec_print_err("remain_bytes larger than ddr size:  remain_bytes 0x%x > ddr_size 0x%x!\n", remain_bytes, image->ddr_size);
        return -ENOMEM;
        /* cov_verified_stop */
    }

    g_images_size[image->etype] = remain_bytes;
    /*split the size to be read to each SECBOOT_BUFLEN bytes.*/
    while (remain_bytes)
    {
        if (remain_bytes > SECBOOT_BUFLEN)
            read_bytes = SECBOOT_BUFLEN;
        else
            read_bytes = remain_bytes;

        readed_bytes = read_file(file_name, skip_offset + file_offset, (u32)read_bytes, (s8 *)SECBOOT_BUFFER);
        if (readed_bytes < 0 || readed_bytes != read_bytes) {
            sec_print_err("read_file %s err: readed_bytes 0x%x\n", file_name, readed_bytes);
            return readed_bytes;
        }

        if(!load_position_offset)
        {
            if( (image->etype == DSP_COLD_PATCH) || (image->etype == MODEM_COLD_PATCH) )
            {
                /* œ«ÕûžögzipžñÊœµÄÑ¹ËõŸµÏñ·ÅÔÚDDR¿ÕŒäœáÊøÎ»ÖÃ */
                load_position_offset = (u32)(image->ddr_size - (u32)remain_bytes);
            }
            else if (image->etype == MODEM)
            {
                load_position_offset = get_img_load_position_offset(image,remain_bytes);
            }
            else if(image->etype == DSP)
            {
                load_position_offset = get_img_load_position_offset(image,remain_bytes);
            }
        }

        if ((!load_position_offset) && (image->etype == MODEM))
            /* œ«ÕûžögzipžñÊœµÄÑ¹ËõŸµÏñ·ÅÔÚDDR¿ÕŒäœáÊøÎ»ÖÃ */
            load_position_offset = (u32)(image->ddr_size - (u32)remain_bytes);

        if ((!load_position_offset) && (image->etype == MODEM_DTB)) {
            /* œ«ÕûžögzipžñÊœµÄÑ¹ËõŸµÏñ·ÅÔÚDDR¿ÕŒäœáÊøÎ»ÖÃ */
            load_position_offset = (u32)(image->ddr_size - (u32)remain_bytes);
        }

        ret = trans_data_to_os(image->etype, image->run_addr,  (void *)(SECBOOT_BUFFER), load_position_offset+file_offset, (u32)read_bytes);
        sec_print_info("trans data ot os: etype 0x%x ,load_offset 0x%x, to secos file_offset 0x%x, bytes 0x%x success\n",
            image->etype, load_position_offset+file_offset, file_offset, read_bytes);

        if (ret)
        {
            /* cov_verified_start */
            sec_print_err("modem image trans to os is failed, error code 0x%x\n", ret);
            return ret;
            /* cov_verified_stop */
        }

        remain_bytes -= read_bytes;
        file_offset += (u32)read_bytes;
    }

    return SEC_OK;
}

static int ccpu_reset(enum SVC_SECBOOT_IMG_TYPE  image)
{
    TEEC_Session *session = &load_session;
    TEEC_Result result;
    TEEC_Operation operation;
    u32 origin;

    operation.started = 1;
    operation.cancel_flag = 0;

    operation.paramTypes =
        (u32)TEEC_PARAM_TYPES(TEEC_VALUE_INPUT, TEEC_NONE, TEEC_NONE, TEEC_NONE);/*lint !e845*/

    operation.params[0].value.a = image;
    result = TEEK_InvokeCommand(
                session,
                SECBOOT_CMD_ID_RESET_IMAGE,
                &operation,
                &origin);
    if (result != TEEC_SUCCESS) {
        /* cov_verified_start */
        sec_print_err("invoke failed, result %#x\n", result);
        /* cov_verified_stop */
    }

    return (int)result;
}

s32 load_image(enum SVC_SECBOOT_IMG_TYPE ecoretype, u32 run_addr, u32 ddr_size)
{
    s32 ret;
    bool is_sec;
    char file_name[LOADM_FILE_NAME_LEN] = {0};
    int readed_bytes;
    struct image_type_name *image;

    ret = get_image(&image, ecoretype, run_addr, ddr_size);
    if(ret)
    {
        /* cov_verified_start */
        sec_print_err("can't find image\n");
        return ret;
        /* cov_verified_stop */
    }

    if(!run_addr)
    {
        run_addr = image->run_addr;
    }

    ret = get_file_name(file_name, image, &is_sec);
    if(ret)
    {
        /* cov_verified_start */
        sec_print_err("can't find image\n");
        return ret;
        /* cov_verified_stop */
    }
    sec_print_info("find file %s, is_sec: %d\n", file_name, is_sec);

    /*load vrl data to sec os*/
    if(is_sec)
    {
        readed_bytes = read_file(file_name, 0, VRL_SIZE, (char*)SECBOOT_BUFFER);
        if(readed_bytes < 0 || readed_bytes != VRL_SIZE)
        {
            sec_print_err("read_file %s error, readed_bytes 0x%x!\n", file_name, readed_bytes);
            ret = readed_bytes;
            goto error;
        }

        ret = trans_vrl_to_os(image->etype, (void *)(SECBOOT_BUFFER), VRL_SIZE);
        if(ret)
        {
            /* cov_verified_start */
            sec_print_err("trans_vrl_to_os error, ret 0x%x!\n", ret);
            goto error;
            /* cov_verified_stop */
        }
        sec_print_err("trans vrl to secos success\n");
    }

    /* load image data to sec os */
    ret = load_data_to_secos(file_name, 0, 0, image, is_sec);
    if(ret)
    {
        /* cov_verified_start */
        sec_print_err("load image %s to secos failed, ret = 0x%x\n", file_name, ret);
        goto error;
        /* cov_verified_stop */
    }
    sec_print_err("load image %s to secos success\n", file_name);

    /*end of trans all data, start verify*/
    ret = verify_soc_image(ecoretype, run_addr);
    if(ret)
    {
        /* cov_verified_start */
        sec_print_err("verify image %s fail, ret = 0x%x\n", file_name, ret);
        goto error;
        /* cov_verified_stop */
    }
    sec_print_err("verify image %s success\n", file_name);

error:

    return ret;
}

void load_modem_cold_patch_image(enum SVC_SECBOOT_IMG_TYPE ecoretype)
{
    s32 ret = 0;
    char file_name[256] = {0};
    enum modem_patch_type patch_type;

    file_name[0] = '\0';
    strncat_s(file_name, LOADM_FILE_NAME_LEN, MODEM_COLD_PATCH_PATH, strnlen(MODEM_COLD_PATCH_PATH, LOADM_FILE_NAME_LEN));
    if(ecoretype == DSP_COLD_PATCH)
    {
        strncat_s(file_name, LOADM_FILE_NAME_LEN, "sec_phy.bin.p", strnlen("sec_phy.bin.p", LOADM_FILE_NAME_LEN));
        patch_type = DSP_PATCH;
    }
    else if(ecoretype == MODEM_COLD_PATCH)
    {
        strncat_s(file_name, LOADM_FILE_NAME_LEN, "sec_balong_modem.bin.p", strnlen("sec_balong_modem.bin.p", LOADM_FILE_NAME_LEN));
        patch_type = CCORE_PATCH;
    }
    else
    {
        sec_print_err("modem patch image type error: %d\n", ecoretype);
        return;
    }

    if(!bsp_access((s8*) file_name, RFILE_RDONLY))
    {
        g_cold_patch_info.modem_patch_info[patch_type].patch_exist = 1;
        if(g_cold_patch_info.modem_update_fail_count < 3)
        {
            ret = load_image(ecoretype, 0, 0);
            g_cold_patch_info.patch_ret_value[patch_type].load_ret_val = ret;
            if(ret)
            {
                sec_print_err("load patch img failed, img_type %d\n",ecoretype);
                g_cold_patch_info.modem_patch_info[patch_type].patch_status = LOAD_PATCH_FAIL;
            }
            else
                g_cold_patch_info.modem_patch_info[patch_type].patch_status = PUT_PATCH;
        }
        else
        {
            sec_print_err("fail_count = %d,not load patch img, img_type %d\n",g_cold_patch_info.modem_update_fail_count,ecoretype);
            g_cold_patch_info.modem_patch_info[patch_type].patch_status = NOT_LOAD_PATCH;
        }
    }
    else
    {
        g_cold_patch_info.modem_patch_info[patch_type].patch_exist = 0;
        sec_print_err("patch img not exist, img_type %d\n",ecoretype);
        g_cold_patch_info.modem_patch_info[patch_type].patch_status = NOT_LOAD_PATCH;
    }

    return;
}
void update_modem_cold_patch_status(enum modem_patch_type epatch_type)
{
    int i = 0;
    g_cold_patch_info.cold_patch_status = 0;

    if(g_cold_patch_info.modem_update_fail_count >= 3)
        return;

    if(g_cold_patch_info.modem_patch_info[epatch_type].patch_status == PUT_PATCH)
        g_cold_patch_info.modem_patch_info[epatch_type].patch_status = PUT_PATCH_SUCESS;

    /*Èç¹û²¹¶¡ŸµÏñ²»ŽæÔÚ»ò²¹¶¡ŸµÏñÎŽŒÓÔØ£¬²¹¶¡µÄ×ŽÌ¬ÎªNOT_LOAD_PATCH£»Èç¹û²¹¶¡ŸµÏñÓëŸµÏñÆŽœÓ³É¹Š£¬²¹¶¡µÄ×ŽÌ¬ÎªPUT_PATCH_SUCESS£»
    Èç¹ûËùÓÐ²¹¶¡ŸµÏñµÄ×ŽÌ¬ÎªNOT_LOAD_PATCH£¬±íÊŸÎŽŒÓÔØµÄ²¹¶¡ŸµÏñ£¬œ«cold_patch_statusÉèÖÃÎª0£»
    Èç¹ûËùÓÐ²¹¶¡ŸµÏñµÄ×ŽÌ¬ÎªNOT_LOAD_PATCH»òPUT_PATCH_SUCESS£šÖÁÉÙÒ»žö²¹¶¡ŸµÏñ×ŽÌ¬ÎªPUT_PATCH_SUCESS£©£¬±íÊŸ²¹¶¡ŸµÏñÓëÔ­ŸµÏñÆŽœÓ³É¹Š£¬œ«cold_patch_statusÉèÖÃÎª1*/
    for(i = 0; i < MAX_PATCH; i++)
    {
        if( (g_cold_patch_info.modem_patch_info[i].patch_status != PUT_PATCH_SUCESS) && (g_cold_patch_info.modem_patch_info[i].patch_status != NOT_LOAD_PATCH) ){
            sec_print_err("modify_modem_cold_patch_status,line = %d, patch_type = 0x%x, patch_status = %d\n",__LINE__,i,g_cold_patch_info.modem_patch_info[i].patch_status);
            return;
        }
    }
    for(i = 0; i < MAX_PATCH; i++)
    {
        if((g_cold_patch_info.modem_patch_info[i].patch_exist) && (g_cold_patch_info.modem_patch_info[i].patch_status != NOT_LOAD_PATCH))
        {
            g_cold_patch_info.cold_patch_status = 1;
            return;
        }
    }
    return;
}

void record_cold_patch_splicing_ret_val(enum modem_patch_type epatch_type,int value)
{
    if(g_cold_patch_info.modem_patch_info[epatch_type].patch_status == PUT_PATCH)
    {
        g_cold_patch_info.patch_ret_value[epatch_type].splice_ret_val = value;
        g_cold_patch_info.modem_update_fail_count = 3;
        g_cold_patch_info.cold_patch_status =0;
    }
}




static int get_dtb_entry(unsigned int modemid, unsigned int num, struct modem_dt_entry_t *dt_entry_ptr, struct modem_dt_entry_t *dt_entry_ccore)
{
    uint32_t i;
    uint8_t sec_id[4]={0};

    sec_id[0] = MODEMID_K_BITS(modemid);
    sec_id[1] = MODEMID_H_BITS(modemid);
    sec_id[2] = MODEMID_M_BITS(modemid);
    sec_id[3] = MODEMID_L_BITS(modemid);

    /* »ñÈ¡ÓëmodemidÆ¥ÅäµÄacore/ccore dt_entry ÖžÕë,žŽÓÃdtctool£¬modem config.dtsÖÐœ«boardidÅäÖÃÎª¶ÔÓŠmodem_idÖµ */
    for (i = 0; i < num; i++)
    {
        if ((dt_entry_ptr->boardid[0] == sec_id[0]) &&
        (dt_entry_ptr->boardid[1] == sec_id[1]) &&
        (dt_entry_ptr->boardid[2] == sec_id[2]) &&
        (dt_entry_ptr->boardid[3] == sec_id[3]))
        {
            sec_print_info("[%d],modemid(0x%x, 0x%x, 0x%x, 0x%x)\n",
                i, dt_entry_ptr->boardid[0], dt_entry_ptr->boardid[1], dt_entry_ptr->boardid[2], dt_entry_ptr->boardid[3]);

            memcpy_s((void *)dt_entry_ccore, sizeof(*dt_entry_ccore), (void *)dt_entry_ptr, sizeof(*dt_entry_ptr));
            break;
        }
        dt_entry_ptr++;
    }

    if(i == num) {
        return -ENOENT;
    }

    return SEC_OK;
}

static s32 load_and_verify_dtb_data(void)
{
    s32 ret, condition;
    u32 modem_id = 0;
    struct modem_dt_table_t *header;
    struct modem_dt_table_t dt_table;
    struct modem_dt_entry_t *dt_entry;
    struct modem_dt_entry_t dt_entry_ptr = {{0}};



    char file_name[LOADM_FILE_NAME_LEN] = {0};
    struct image_type_name *image;
    bool is_sec;
    u32 offset = 0;
    int readed_bytes;

    header = &dt_table;
    ret = get_image(&image, MODEM_DTB,0,0);
    if(ret)
    {
        sec_print_err("can't find image\n");
        return ret;
    }

    ret = get_file_name(file_name, image, &is_sec);
    if(ret)
    {
        sec_print_err("can't find image\n");
        return ret;
    }
    sec_print_info("find file %s, is_sec: %d\n", file_name, is_sec);

    /* °²È«°æ±ŸÌø¹ýsec VRLÍ· */
    if(is_sec)
    {
        offset = VRL_SIZE;
    }

    /*get the head*/
    readed_bytes = read_file(file_name, offset, (unsigned int)sizeof(struct modem_dt_table_t), (char*)(header));
    condition = readed_bytes < 0 || readed_bytes != (int)sizeof(struct modem_dt_table_t);
    if (condition)
    {
       sec_print_err("fail to read the head of modem dtb image, readed_bytes(0x%x) != size(0x%lx).\n", readed_bytes, sizeof(struct modem_dt_table_t));
       ret = readed_bytes;
       return ret;
    }

    /*get the entry*/
    dt_entry  = (struct modem_dt_entry_t *)kzalloc((size_t)((sizeof(struct modem_dt_entry_t)*(header->num_entries))), GFP_KERNEL);
    if(NULL == dt_entry)
    {
       sec_print_err("dtb header malloc fail\n");
       return -ENOMEM;
    }

    offset += sizeof(struct modem_dt_table_t);

    readed_bytes = read_file(file_name, offset, (unsigned int)(sizeof(struct modem_dt_entry_t)*(header->num_entries)), (char*)dt_entry);
    condition = readed_bytes < 0 || readed_bytes != (int)(sizeof(struct modem_dt_entry_t)*(header->num_entries));
    if (condition)
    {
       sec_print_err("fail to read the head of modem dtb entry, readed_bytes(0x%x) != size(0x%lx).\n", readed_bytes, (sizeof(struct modem_dt_entry_t)*(header->num_entries)));
       ret = readed_bytes;
       goto err_out;
    }
    offset -= sizeof(struct modem_dt_table_t);
    /* ÐèÒªmaskµôÉäÆµ¿Û°åIDºÅ»òmodemidµÄbit[9:0] */

    if (bsp_get_version_info() != NULL)
        modem_id = bsp_get_version_info()->board_id_udp_masked;
    else
        goto err_out;
    sec_print_err("modem_id 0x%x \n", modem_id);

    memset_s((void *)&dt_entry_ptr, sizeof(dt_entry_ptr), 0, sizeof(dt_entry_ptr));
    ret = get_dtb_entry(modem_id, header->num_entries, dt_entry,  &dt_entry_ptr);
    if (ret)
    {
        sec_print_err("fail to get_dtb_entry\n");
        goto err_out;
    }

    /* °²È«°æ±ŸÇÒÊ¹ÄÜÁËÇ©Ãû */
    if(is_sec && 0 != dt_entry_ptr.vrl_size)
    {
        /*load vrl data to sec os*/
        if(dt_entry_ptr.vrl_size > SECBOOT_BUFLEN)
        {
            sec_print_err("modem dtb vrl_size too large %d\n", dt_entry_ptr.vrl_size);
            ret = -ENOMEM;
            goto err_out;
        }
        sec_print_info("modem dtb vrl_offset %d, vrl_size %d\n", dt_entry_ptr.vrl_offset,dt_entry_ptr.vrl_size);

        readed_bytes = read_file(file_name, offset + dt_entry_ptr.vrl_offset,dt_entry_ptr.vrl_size, (char*)SECBOOT_BUFFER);
        condition = readed_bytes < 0 || (u32)readed_bytes != dt_entry_ptr.vrl_size;
        if (condition)
        {
            sec_print_err("fail to read the dtb vrl\n");
            ret = readed_bytes;
            goto err_out;
        }

        ret = trans_vrl_to_os(image->etype, (void *)(SECBOOT_BUFFER), VRL_SIZE);
        if(ret)
        {
            sec_print_err("trans_vrl_to_os error, ret 0x%x!\n", ret);
            goto err_out;
        }
        sec_print_err("trans vrl to secos success\n");

    }

    /* load image data to sec os */
    ret = load_data_to_secos(file_name, offset + dt_entry_ptr.dtb_offset, dt_entry_ptr.dtb_size, image, is_sec);
    if(ret)
    {
        sec_print_err("load image %s to secos failed, ret = 0x%x\n", file_name, ret);
        goto err_out;
    }
    sec_print_err("load image %s to secos success\n", file_name);

    if(is_sec)
    {
        ret = verify_soc_image(image->etype, image->run_addr);
        if (ret)
        {
            sec_print_err("fail to verify modem dtb image\n");
            goto err_out;
        }
    }
    sec_print_err("verify modem dtb success\n");
err_out:
    kfree(dt_entry);
    return ret;
}

/*****************************************************************************
 º¯ Êý Ãû  : ModemÏà¹ØŸµÏñŒÓÔØœÓ¿Ú
 ¹ŠÄÜÃèÊö  : ModemÏà¹ØŸµÏñŒÓÔØœÓ¿Ú
 ÊäÈë²ÎÊý  : ÎÞ
 Êä³ö²ÎÊý  : ÎÞ
 ·µ »Ø Öµ  : ³É¹Š·µ»ØOK,Ê§°Ü·µ»ØERROR
*****************************************************************************/
int bsp_load_modem_images(void)
{
    int ret;

    u32 cold_patch_stamp[2][2] = {{0,0}};

    mutex_lock(&load_proc_lock);

    ret = TEEK_init();
    if(ret)
    {
        mutex_unlock(&load_proc_lock);
        sec_print_err("TEEK_init failed! ret %#x\n" ,ret);
        return ret;
    }

    ret = ccpu_reset(MODEM);
    if(ret)
    {
        mutex_unlock(&load_proc_lock);
        sec_print_err("ccpu_reset failed, ret %#x\n", ret);
        return ret;
    }

    ret = bsp_nvm_mreset_load();
    if(ret)
    {
        goto error;
    }


    cold_patch_stamp[0][0] = bsp_get_slice_value();
    if(bsp_nvem_cold_patch_read(&g_cold_patch_info))
        memset_s(&g_cold_patch_info,sizeof(g_cold_patch_info),0,sizeof(g_cold_patch_info));
    load_modem_cold_patch_image(DSP_COLD_PATCH);

    ret = load_image(DSP, 0, 0);
    if(ret)
    {
        goto cold_patch_error;
    }


    cold_patch_stamp[0][1] = bsp_get_slice_value();
    update_modem_cold_patch_status(DSP_PATCH);


    ret = load_and_verify_dtb_data();
    if(ret)
    {
        goto error;
    }

    cold_patch_stamp[1][0] = bsp_get_slice_value();
    load_modem_cold_patch_image(MODEM_COLD_PATCH);

    ret = load_image(MODEM, 0, 0);
    if(ret)
    {
        record_cold_patch_splicing_ret_val(CCORE_PATCH,ret);
        goto error;
    }

    cold_patch_stamp[1][1] = bsp_get_slice_value();
    update_modem_cold_patch_status(CCORE_PATCH);


cold_patch_error:
    if(ret)
    {
        record_cold_patch_splicing_ret_val(DSP_PATCH,ret);
    }

error:
    TEEK_uninit();
    (void)bsp_nvem_cold_patch_write(&g_cold_patch_info);
    printk(KERN_INFO "dsp_cold_patch loads start at 0x%x, splice end at 0x%x,modem_cold_patch loads start at 0x%x, splice end at 0x%x\n",
        cold_patch_stamp[0][0],cold_patch_stamp[0][1],cold_patch_stamp[1][0],cold_patch_stamp[1][1]);

    mutex_unlock(&load_proc_lock);

    return ret;
}

/*****************************************************************************
 º¯ Êý Ãû  : was.img¡¢tas.imgµÈ¶¯Ì¬ŒÓÔØŸµÏñœÓ¿Ú
 ¹ŠÄÜÃèÊö  : ModemÏà¹ØŸµÏñŒÓÔØœÓ¿Ú
 ÊäÈë²ÎÊý  : ÎÞ
 Êä³ö²ÎÊý  : ÎÞ
 ·µ »Ø Öµ  : ³É¹Š·µ»ØOK,Ê§°Ü·µ»ØERROR
*****************************************************************************/
int bsp_load_modem_single_image(enum SVC_SECBOOT_IMG_TYPE ecoretype, u32 run_addr, u32 ddr_size)
{
    int ret;

    mutex_lock(&load_proc_lock);

    ret = TEEK_init();
    if(ret)
    {
        mutex_unlock(&load_proc_lock);
        sec_print_err("TEEK_InitializeContext failed!\n");
        return ret;
    }

    ret = load_image(ecoretype, run_addr, ddr_size);
    if(ret)
    {
        goto error;
    }

error:
    TEEK_uninit();

    mutex_unlock(&load_proc_lock);

    return ret;
}

int __init bsp_load_image_init(void)
{

    SECBOOT_BUFFER = (u8*)kmalloc(SECBOOT_BUFLEN, GFP_KERNEL);
    if (!SECBOOT_BUFFER) {
        sec_print_err("fail to malloc secboot buffer\n");
        return -1;
    }

    return 0;
}

core_initcall(bsp_load_image_init);


